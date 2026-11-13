import { PanelExtensionContext, SettingsTreeAction, Topic } from "@lichtblick/suite";
import { CSSProperties, ReactElement, useCallback, useEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import { numberFrom, panelStyle, settingsBoolean, settingsNumber, stringFrom, updateSettingsEditor } from "./panelUtils";
import { envelope, TimingSample, TopicTiming } from "./topicTiming";

type State = { hardware: boolean; all: boolean; selected: string; windowSec: number;
  staleSec: number; basis: "receive" | "header"; expectedMs: number };
const inputStyle: CSSProperties = { background: "transparent", color: "inherit", border: "1px solid #666", borderRadius: 3, padding: 4 };
const cell: CSSProperties = { padding: "5px 8px", textAlign: "right", whiteSpace: "nowrap", borderBottom: "1px solid #444" };
const fmt = (value: number | undefined, unit = "", digits = 1): string => value == undefined ? "—" : `${value.toFixed(digits)}${unit}`;
const ms = (time: { sec: number; nsec: number }): number => time.sec * 1000 + time.nsec / 1e6;
function headerMs(message: unknown): number | undefined {
  const stamp = (message as { header?: { stamp?: { sec?: unknown; nanosec?: unknown; nsec?: unknown } } } | undefined)?.header?.stamp;
  const ns = stamp?.nanosec ?? stamp?.nsec;
  if (typeof stamp?.sec !== "number" || typeof ns !== "number" || (stamp.sec === 0 && ns === 0)) return undefined;
  return stamp.sec * 1000 + ns / 1e6;
}
function initial(context: PanelExtensionContext): State {
  const p = (context.initialState ?? {}) as Partial<State>;
  return { hardware: false, all: false,
    selected: stringFrom(p.selected, ""),
    windowSec: Math.min(120, Math.max(1, numberFrom(p.windowSec, 30))),
    staleSec: Math.min(60, Math.max(0.1, numberFrom(p.staleSec, 2))),
    basis: p.basis === "header" ? "header" : "receive", expectedMs: Math.max(0, numberFrom(p.expectedMs, 0)) };
}
function PeriodGraph({ samples, now, windowMs, expectedMs }: {
  samples: readonly TimingSample[]; now: number; windowMs: number; expectedMs: number;
}): ReactElement {
  const [hover, setHover] = useState<TimingSample>();
  const visible = samples.filter((p) => p.at >= now - windowMs && p.at <= now && p.period != undefined);
  const points = envelope(visible);
  const ceiling = Math.max(1, expectedMs, ...points.map((p) => p.period ?? 0)) * 1.1;
  const x = (at: number): number => 58 + (at - (now - windowMs)) / windowMs * 710;
  const y = (value: number): number => 210 - value / ceiling * 180;
  return <div style={{ position: "relative" }}>
    <svg viewBox="0 0 800 250" role="img" aria-label="Message period in milliseconds over time" style={{ display: "block", width: "100%", minHeight: 160 }}
      onMouseLeave={() => setHover(undefined)} onMouseMove={(event) => {
        const matrix = event.currentTarget.getScreenCTM();
        if (!matrix) return;
        const cursor = event.currentTarget.createSVGPoint();
        cursor.x = event.clientX; cursor.y = event.clientY;
        const at = now - windowMs + (cursor.matrixTransform(matrix.inverse()).x - 58) / 710 * windowMs;
        let closest: TimingSample | undefined;
        for (const point of points) if (!closest || Math.abs(point.at - at) < Math.abs(closest.at - at)) closest = point;
        setHover(closest);
      }}>
      {[0, 0.25, 0.5, 0.75, 1].map((f) => <g key={f}>
        <line x1="58" x2="768" y1={y(ceiling * f)} y2={y(ceiling * f)} stroke="#555" strokeWidth="0.5" />
        <text x="51" y={y(ceiling * f) + 4} textAnchor="end" fill="currentColor" fontSize="11">{(ceiling * f).toFixed(1)}</text>
        <text x={58 + f * 710} y="232" textAnchor="middle" fill="currentColor" fontSize="11">{(-windowMs * (1 - f) / 1000).toFixed(0)}s</text>
      </g>)}
      <text x="58" y="16" fill="currentColor" fontSize="12">Period / ms</text>
      {expectedMs > 0 && <line x1="58" x2="768" y1={y(expectedMs)} y2={y(expectedMs)} stroke="#eab34d" strokeDasharray="5 4" />}
      <polyline fill="none" stroke="#36c7e7" strokeWidth="1.5" points={points.map((p) => `${x(p.at)},${y(p.period ?? 0)}`).join(" ")} />
      {points.length === 1 && points[0] && <circle cx={x(points[0].at)} cy={y(points[0].period ?? 0)} r="3" fill="#36c7e7" />}
      {hover && <circle cx={x(hover.at)} cy={y(hover.period ?? 0)} r="4" fill="#ffffff" />}
    </svg>
    {hover && <div style={{ position: "absolute", right: 12, top: 0, fontSize: 12, background: "#29292d", padding: "3px 6px", pointerEvents: "none" }}>
      {fmt((hover.at - now) / 1000, " s", 2)} · {fmt(hover.period, " ms", 3)}
    </div>}

  </div>;
}

function TopicMonitor({ context }: { context: PanelExtensionContext }): ReactElement {
  const [state, setState] = useState(() => initial(context));
  const stateRef = useRef(state); stateRef.current = state;
  const [topics, setTopics] = useState<readonly Topic[]>([]);
  const [filter, setFilter] = useState("");
  const [now, setNow] = useState(0);
  const [, redraw] = useState(0);
  const histories = useRef(new Map<string, TopicTiming>());
  const clock = useRef({ source: 0, wall: performance.now() });
  const subscriptionKey = useRef("");
  const catalogKey = useRef("");
  const [limited, setLimited] = useState(false);

  const actionHandler = useCallback((action: SettingsTreeAction) => {
    if (action.action === "perform-node-action" && action.payload.id === "reset") {
      histories.current.clear(); setLimited(false); redraw((n) => n + 1);
      return;
    }
    if (action.action !== "update") return;
    const key = action.payload.path[action.payload.path.length - 1];
    const value = action.payload.value;
    if ((key === "hardware" || key === "all") && typeof value === "boolean") {
      setState((s) => ({ ...s, [key]: value }));
    } else if (key === "basis" && (value === "receive" || value === "header")) {
      setState((s) => ({ ...s, basis: value }));
    } else if ((key === "windowSec" || key === "staleSec" || key === "expectedMs") && typeof value === "number" && Number.isFinite(value)) {
      const min = key === "windowSec" ? 1 : key === "staleSec" ? 0.1 : 0;
      const max = key === "windowSec" ? 120 : key === "staleSec" ? 60 : 60000;
      setState((s) => ({ ...s, [key]: Math.min(max, Math.max(min, value)) }));
    }
  }, []);
  useEffect(() => {
    updateSettingsEditor(context, actionHandler, {
      subscriptions: { label: "Subscriptions", icon: "Settings", fields: {
        hardware: { ...settingsBoolean("Subscribe to /hardware topics", state.hardware), disabled: state.all },
        all: settingsBoolean("Subscribe to all topics", state.all,
          "Temporarily receive every topic, including large messages. Bulk subscriptions start off when the panel is reopened."),
      } },
      timing: { label: "Timing", icon: "Settings", fields: {
        windowSec: settingsNumber("Window (s)", state.windowSec, { min: 1, max: 120 }),
        staleSec: settingsNumber("Stale after (s)", state.staleSec, { min: 0.1, max: 60, step: 0.1,
          help: "Topics without new messages turn amber and show 0 Hz after this timeout." }),
        basis: { input: "select", label: "Period timestamps", value: state.basis, options: [
          { label: "Receive time", value: "receive" }, { label: "header.stamp", value: "header" },
        ], help: "Frequency and age always use reception time. Header mode measures source timestamp spacing." },
        expectedMs: settingsNumber("Reference period (ms)", state.expectedMs, { min: 0, max: 60000,
          help: "Optional reference line for the selected topic. 0 hides it." }),
      }, actions: [{ type: "action", id: "reset", label: "Reset measurements" }] },
    });
  }, [context, state, actionHandler]);

  // Bulk reception is temporary: reopening the panel starts with both scopes off.
  useEffect(() => { context.saveState({ ...state, hardware: false, all: false }); }, [context, state]);
  useEffect(() => { histories.current.clear(); setLimited(false); }, [state.basis]);
  // Preserve histories while selecting a different topic in the same bulk subscription.
  useEffect(() => {
    const names = topics.filter((t) => state.all || (state.hardware && t.name.startsWith("/hardware/")) || t.name === state.selected).map((t) => t.name).sort();
    const key = JSON.stringify(names);
    if (key !== subscriptionKey.current) {
      const active = new Set(names);
      for (const topic of histories.current.keys()) if (!active.has(topic)) histories.current.delete(topic);
      context.subscribe(names.map((topic) => ({ topic })));
      subscriptionKey.current = key;
    }
  }, [context, state.all, state.hardware, state.basis, state.selected, topics]);

  useEffect(() => {
    context.setDefaultPanelTitle("Topic Monitor");
    context.watch("topics"); context.watch("currentFrame"); context.watch("currentTime"); context.watch("didSeek");
    context.onRender = (render, done) => {
      const sourceNow = render.currentTime ? ms(render.currentTime) : undefined;
      if (render.didSeek || (sourceNow != undefined && sourceNow < clock.current.source)) histories.current.clear();
      if (sourceNow != undefined) clock.current = { source: sourceNow, wall: performance.now() };
      if (render.topics) {
        const key = JSON.stringify(render.topics.map((t) => [t.name, t.schemaName]));
        if (key !== catalogKey.current) {
          catalogKey.current = key;
          setTopics(render.topics.map((t) => ({ ...t })).sort((a, b) => a.name.localeCompare(b.name)));
        }
      }
      const current = stateRef.current;
      for (const event of render.currentFrame ?? []) {
        if (!(current.all || (current.hardware && event.topic.startsWith("/hardware/")) || event.topic === current.selected)) continue;
        let history = histories.current.get(event.topic);
        if (!history) {
          if (histories.current.size >= 256) { setLimited(true); continue; }
          history = new TopicTiming(); histories.current.set(event.topic, history);
        }
        const at = ms(event.receiveTime);
        const stamp = current.basis === "receive" ? at : headerMs(event.message);
        history.add(at, stamp);
        // Release expired samples as well as imposing a per-topic count bound.
        if (history.count % 100 === 0) history.prune(at - current.windowSec * 1000);
      }
      done();
    };
    const timer = window.setInterval(() => {
      const time = clock.current.source + performance.now() - clock.current.wall;
      for (const history of histories.current.values()) history.prune(time - stateRef.current.windowSec * 1000);
      setNow(time);
    }, 250);
    return () => { window.clearInterval(timer); context.onRender = undefined; context.unsubscribeAll(); };
  }, [context]);

  const selected = histories.current.get(state.selected);
  const summary = selected?.summarize(now, state.windowSec * 1000, state.staleSec * 1000, state.expectedMs);
  const observing = (name: string): boolean => state.all || (state.hardware && name.startsWith("/hardware/")) || name === state.selected;
  const toggleTopic = (name: string): void => {
    setState((s) => ({ ...s, selected: s.selected === name ? "" : name, expectedMs: 0 }));
  };
  return <div style={{ ...panelStyle, gap: 8 }}>
    {limited && <div role="status" style={{ color: "#efb84b", fontSize: 12 }} title="History is limited to 256 topics. Narrow the subscription scope in panel settings and reset measurements.">History limit reached</div>}
    <input aria-label="Filter topics" placeholder="Filter topic name or type…" value={filter} onChange={(e) => setFilter(e.target.value)} style={inputStyle} />
    <div style={{ overflow: "auto", minHeight: state.selected ? 100 : 0, maxHeight: state.selected ? "42%" : undefined, flex: "1 1 220px" }}>
      <table style={{ borderCollapse: "separate", borderSpacing: 0, width: "100%", fontVariantNumeric: "tabular-nums" }}>
        <thead style={{ position: "sticky", top: 0, zIndex: 2, backgroundColor: "#29292d" }}>
          <tr>{["Topic", "Message type", "Hz"].map((label, index) => <th key={label} scope="col" style={{ ...cell,
            textAlign: index === 2 ? "right" : "left", backgroundColor: "#29292d",
            position: "sticky", top: 0, zIndex: 2,
          }}>{label}</th>)}</tr>
        </thead>
        <tbody>{topics.filter((t) => `${t.name} ${t.schemaName ?? ""}`.toLowerCase().includes(filter.toLowerCase())).map((topic) => {
          const history = histories.current.get(topic.name);
          const stats = history?.rate(now, state.windowSec * 1000, state.staleSec * 1000);
          const active = observing(topic.name);
          const status = !active ? "Not subscribed" : !history ? "Waiting for messages" : stats?.stale ? "No recent messages" : "Receiving";
          return <tr key={topic.name} title={`${topic.schemaName ?? ""} · ${status}`}
            tabIndex={0} aria-selected={topic.name === state.selected} aria-expanded={topic.name === state.selected}
            onClick={() => toggleTopic(topic.name)}
            onKeyDown={(event) => {
              if (event.key === "Enter" || event.key === " ") { event.preventDefault(); toggleTopic(topic.name); }
            }} style={{ cursor: "pointer",
            background: topic.name === state.selected ? "#414145" : undefined,
            color: stats?.stale && active ? "#efb84b" : undefined,
            opacity: !active || !history ? 0.45 : 1,
          }}>
            <td style={{ ...cell, textAlign: "left" }}>{topic.name}</td>
            <td style={{ ...cell, textAlign: "left", maxWidth: 240, overflow: "hidden", textOverflow: "ellipsis" }} title={topic.schemaName}>{topic.schemaName ?? "—"}</td>
            <td style={cell}>{active ? fmt(stats?.hz, "", 2) : "—"}</td>
          </tr>;
        })}</tbody>
      </table>
      {topics.length === 0 && <div>Waiting for topic discovery…</div>}
    </div>
    {state.selected !== "" && <section style={{ flex: "0 0 auto", borderTop: "1px solid #444", paddingTop: 10 }}>
      <div style={{ display: "flex", justifyContent: "space-between", flexWrap: "wrap", alignItems: "baseline", gap: 8 }}>
        <strong style={{ fontSize: 12, overflowWrap: "anywhere" }}>{state.selected || "Select a topic"}</strong>
        <span style={{ fontSize: 12, opacity: 0.75 }}>Average {fmt(summary?.mean, " ms")}</span>
      </div>
      <PeriodGraph key={`${state.selected}:${state.basis}`} samples={selected?.samples ?? []} now={now} windowMs={state.windowSec * 1000} expectedMs={state.expectedMs} />
      {state.basis === "header" && selected && !selected.samples.some((sample) => sample.period != undefined) &&
        <div style={{ fontSize: 12, opacity: 0.6 }}>Waiting for valid header timestamps…</div>}
      {selected?.truncated && <div style={{ color: "#efb84b", fontSize: 12 }} title="History is capped at 5,000 messages per topic and may cover less than the configured window.">History limited</div>}

    </section>}
  </div>;
}
export function initTopicMonitorPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<TopicMonitor context={context} />);
  return () => root.unmount();
}
