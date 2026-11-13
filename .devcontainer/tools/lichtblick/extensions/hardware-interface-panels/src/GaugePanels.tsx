import { PanelExtensionContext, SettingsTreeAction } from "@lichtblick/suite";
import { CSSProperties, ReactElement, useCallback, useEffect, useState } from "react";
import { createRoot } from "react-dom/client";

import {
  boolFrom, clamp, handleSettingsAction, numberFrom, settingsBoolean,
  settingsNumber, settingsString, stringFrom, updateSettingsEditor,
} from "./panelUtils";

type Kind = "rpm" | "temperature" | "thermometer" | "steering" | "electrical";
type Source = {
  label: string; topic: string; field: string; unit: string;
  min: number; max: number; decimals: number; scale: number;
  color: string; thresholds: boolean; warning: number; critical: number;
};
type State = {
  primary: Source; secondary: Source; timeoutMs: number;
  vertical: boolean; rotationScale: number; invertRotation: boolean;
};
type Reading = { value: number; time: number } | undefined;
const surface: CSSProperties = {
  background: "transparent", color: "#ececf0", font: "13px system-ui, sans-serif",
  boxSizing: "border-box", height: "100%", width: "100%", padding: "12px 16px",
  display: "flex", flexDirection: "column", justifyContent: "center", gap: 12,
  overflow: "auto", minHeight: 0,
};
const muted = "#939eae";
const track = "#303a47";
function source(label: string, topic: string, field: string, unit: string, min: number, max: number): Source {
  return { label, topic, field, unit, min, max, decimals: 1, scale: 1,
    color: "#36c7e7", thresholds: false, warning: max * 0.75, critical: max * 0.9 };
}
export function gaugeDefaults(kind: Kind): State {
  const voltage = source("Voltage", "/hardware/telemetry/motor", "input_voltage", "V", 9, 15);
  let primary = source("RPM", "/hardware/measure/motor_feedback", "rpm", "RPM", -12000, 12000);
  let secondary = voltage;
  if (kind === "temperature" || kind === "thermometer") primary = source("Temperature", "/hardware/telemetry/motor", "fet_temperature", "°C", 0, 120);
  if (kind === "steering") {
    primary = source("Actual", "/hardware/measure/steering_angle", "data", "°", -30, 30);
    secondary = source("Target", "/hardware/command/steering_angle", "angle", "°", -30, 30);
  }
  if (kind === "electrical") primary = source("Current", "/hardware/telemetry/motor", "input_current", "A", -20, 80);
  if (kind === "rpm") primary.decimals = 0;
  return { primary, secondary, timeoutMs: kind === "temperature" || kind === "thermometer" ? 10000 : 4000,
    vertical: false, rotationScale: 1, invertRotation: false };
}
function restoreSource(saved: Partial<Source> | undefined, fallback: Source): Source {
  const p = saved ?? {};
  return {
    label: stringFrom(p.label, fallback.label), topic: stringFrom(p.topic, fallback.topic),
    field: stringFrom(p.field, fallback.field), unit: stringFrom(p.unit, fallback.unit),
    min: numberFrom(p.min, fallback.min), max: numberFrom(p.max, fallback.max),
    decimals: clamp(Math.trunc(numberFrom(p.decimals, fallback.decimals)), 0, 6),
    scale: numberFrom(p.scale, fallback.scale), color: stringFrom(p.color, fallback.color),
    thresholds: boolFrom(p.thresholds, fallback.thresholds),
    warning: numberFrom(p.warning, fallback.warning), critical: numberFrom(p.critical, fallback.critical),
  };
}
function initialState(context: PanelExtensionContext, kind: Kind): State {
  const fallback = gaugeDefaults(kind);
  const p = (context.initialState ?? {}) as Partial<State>;
  return { primary: restoreSource(p.primary, fallback.primary), secondary: restoreSource(p.secondary, fallback.secondary),
    timeoutMs: Math.max(0, numberFrom(p.timeoutMs, fallback.timeoutMs)),
    vertical: boolFrom(p.vertical, fallback.vertical),
    rotationScale: numberFrom(p.rotationScale, fallback.rotationScale),
    invertRotation: boolFrom(p.invertRotation, fallback.invertRotation) };
}
/** Simple dot paths, including array indices (e.g. values[0]). */
export function numericField(message: unknown, path: string): number | undefined {
  const segments = path.replace(/\[(\d+)\]/g, ".$1").split(".").filter(Boolean);
  let value = message;
  for (const key of segments) {
    if (value == undefined || typeof value !== "object" || !Object.prototype.hasOwnProperty.call(value, key)) return undefined;
    value = (value as Record<string, unknown>)[key];
  }
  return typeof value === "number" && Number.isFinite(value) ? value : undefined;
}
export function fraction(value: number, min: number, max: number): number {
  return max > min ? clamp((value - min) / (max - min), 0, 1) : 0;
}
function tone(config: Source, value: number | undefined): string {
  if (value == undefined) return muted;
  if (config.thresholds && value >= config.critical) return "#ef6a69";
  if (config.thresholds && value >= config.warning) return "#efb84b";
  return config.color;
}
function formatted(value: number | undefined, config: Source): string {
  return value == undefined ? "—" : value.toFixed(clamp(Math.trunc(config.decimals), 0, 6));
}
function valueLabel(value: number | undefined, config: Source, size = 36): ReactElement {
  return <div style={{ fontSize: size, fontWeight: 600, fontVariantNumeric: "tabular-nums", lineHeight: 1.15 }}>
    {formatted(value, config)} <span style={{ fontSize: "0.45em", color: muted, fontWeight: 400 }}>{config.unit}</span>
  </div>;
}
function validRange(config: Source): boolean { return config.max > config.min; }
function sourceSettings(config: Source) {
  return {
    label: settingsString("Label", config.label), topic: settingsString("Topic", config.topic),
    field: settingsString("Field", config.field, "Dot path, e.g. input_voltage or values[0]."),
    unit: settingsString("Unit", config.unit), min: settingsNumber("Minimum", config.min),
    max: settingsNumber("Maximum", config.max), decimals: settingsNumber("Decimals", config.decimals, { min: 0, max: 6, step: 1 }),
    scale: settingsNumber("Value scale", config.scale), color: settingsString("Color", config.color),
    thresholds: settingsBoolean("Enable thresholds", config.thresholds),
    warning: settingsNumber("Warning above", config.warning), critical: settingsNumber("Critical above", config.critical),
  };
}
function Scale({ config, value, target }: { config: Source; value: number | undefined; target?: number }): ReactElement {
  if (!validRange(config)) return <div style={{ color: "#efb84b" }}>Maximum must exceed minimum</div>;
  const start = fraction(clamp(0, config.min, config.max), config.min, config.max) * 100;
  const end = value == undefined ? start : fraction(value, config.min, config.max) * 100;
  const marker = (position: number, color: string, width = 2): ReactElement => <span style={{
    position: "absolute", left: `${position}%`, top: -4, height: 20, width, background: color, transform: "translateX(-50%)",
  }} />;
  return <div style={{ padding: "6px 3px 0" }}>
    <div style={{ height: 12, position: "relative", background: track, borderRadius: 2 }}>
      <div style={{ position: "absolute", height: "100%", left: `${Math.min(start, end)}%`, width: `${Math.abs(end - start)}%`, background: tone(config, value), borderRadius: 2 }} />
      {config.min < 0 && config.max > 0 && marker(start, muted, 1)}
      {config.thresholds && config.warning >= config.min && config.warning <= config.max && marker(fraction(config.warning, config.min, config.max) * 100, "#efb84b")}
      {config.thresholds && config.critical >= config.min && config.critical <= config.max && marker(fraction(config.critical, config.min, config.max) * 100, "#ef6a69")}
      {value != undefined && marker(end, tone(config, value), 3)}
      {target != undefined && <span style={{ position: "absolute", left: `${fraction(target, config.min, config.max) * 100}%`, top: -7, width: 10, height: 24, border: "2px solid #ececf0", borderRadius: 2, transform: "translateX(-50%)" }} />}
    </div>
    <div style={{ display: "flex", justifyContent: "space-between", color: muted, fontSize: 11, marginTop: 8 }}>
      <span>{config.min}{config.unit}</span><span>{config.max}{config.unit}</span>
    </div>
  </div>;
}
function Linear({ config, value }: { config: Source; value: number | undefined }): ReactElement {
  return <div style={{ display: "flex", flexDirection: "column", gap: 12 }}>
    <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline", gap: 12 }}>
      <span style={{ color: muted }}>{config.label}</span>{valueLabel(value, config)}
    </div>
    <Scale config={config} value={value} />
  </div>;
}
function Ring({ config, value }: { config: Source; value: number | undefined }): ReactElement {
  const progress = value == undefined ? 0 : fraction(value, config.min, config.max);
  const color = tone(config, value);
  return <svg viewBox="0 0 300 260" role="img" aria-label={`${config.label}: ${formatted(value, config)} ${config.unit}`} style={{ width: "100%", maxHeight: "100%", minHeight: 130 }}>
    <path d="M 65.15 224.85 A 120 120 0 1 1 234.85 224.85" fill="none" stroke={track} strokeWidth="12" pathLength="100" />
    <path d="M 65.15 224.85 A 120 120 0 1 1 234.85 224.85" fill="none" stroke={color} strokeWidth="12" pathLength="100" strokeDasharray={`${progress * 100} 100`} />
    <text x="150" y="133" textAnchor="middle" fill={value == undefined ? muted : "#ececf0"} fontSize="43" fontWeight="600">{formatted(value, config)}</text>
    <text x="150" y="160" textAnchor="middle" fill={muted} fontSize="18">{config.unit}</text>
    <text x="150" y="187" textAnchor="middle" fill={muted} fontSize="12">{config.label}</text>
    <text x="55" y="250" textAnchor="middle" fill={muted} fontSize="12">{config.min}</text>
    <text x="245" y="250" textAnchor="middle" fill={muted} fontSize="12">{config.max}</text>
  </svg>;
}
function VerticalTemperature({ config, value }: { config: Source; value: number | undefined }): ReactElement {
  const fill = value == undefined ? 0 : fraction(value, config.min, config.max) * 150;
  return <svg viewBox="0 0 300 230" style={{ width: "100%", maxHeight: "100%", minHeight: 120 }} role="img" aria-label={config.label}>
    <rect x="65" y="30" width="18" height="150" rx="3" fill={track} />
    <rect x="65" y={180 - fill} width="18" height={fill} rx="2" fill={tone(config, value)} />
    <text x="53" y="34" textAnchor="end" fill={muted} fontSize="12">{config.max}</text>
    <text x="53" y="184" textAnchor="end" fill={muted} fontSize="12">{config.min}</text>
    {config.thresholds && [config.warning, config.critical].map((threshold, i) => threshold >= config.min && threshold <= config.max &&
      <line key={i} x1="60" x2="91" y1={180 - fraction(threshold, config.min, config.max) * 150} y2={180 - fraction(threshold, config.min, config.max) * 150} stroke={i === 0 ? "#efb84b" : "#ef6a69"} strokeWidth="2" />)}
    <text x="177" y="110" textAnchor="middle" fill="#ececf0" fontSize="40" fontWeight="600">{formatted(value, config)}</text>
    <text x="177" y="139" textAnchor="middle" fill={muted} fontSize="18">{config.unit}</text>
    <text x="150" y="218" textAnchor="middle" fill={muted} fontSize="12">{config.label}</text>
  </svg>;
}
function Thermometer({ config, value }: { config: Source; value: number | undefined }): ReactElement {
  const fill = value == undefined ? 0 : fraction(value, config.min, config.max) * 142;
  const color = tone(config, value);
  return <svg viewBox="0 0 300 230" style={{ width: "100%", maxHeight: "100%", minHeight: 140 }} role="img" aria-label={`${config.label}: ${formatted(value, config)} ${config.unit}`}>
    <rect x="66" y="26" width="22" height="162" rx="11" fill="transparent" stroke={track} strokeWidth="3" />
    <circle cx="77" cy="188" r="20" fill="transparent" stroke={track} strokeWidth="3" />
    <rect x="71" y={174 - fill} width="12" height={fill + (value == undefined ? 0 : 14)} rx="3" fill={color} />
    <circle cx="77" cy="188" r="14" fill={value == undefined ? track : color} />
    {[config.min, (config.min + config.max) / 2, config.max].map((tick, i) => {
      const y = 174 - fraction(tick, config.min, config.max) * 142;
      return <g key={i}><line x1="94" x2="102" y1={y} y2={y} stroke={muted} /><text x="107" y={y + 4} fill={muted} fontSize="11">{Number(tick.toFixed(2))}</text></g>;
    })}
    {config.thresholds && [config.warning, config.critical].map((threshold, i) => threshold >= config.min && threshold <= config.max &&
      <line key={i} x1="62" x2="91" y1={174 - fraction(threshold, config.min, config.max) * 142} y2={174 - fraction(threshold, config.min, config.max) * 142} stroke={i === 0 ? "#efb84b" : "#ef6a69"} strokeWidth="2" />)}
    <text x="207" y="114" textAnchor="middle" fill={value == undefined ? muted : "#ececf0"} fontSize="40" fontWeight="600">{formatted(value, config)}</text>
    <text x="207" y="143" textAnchor="middle" fill={muted} fontSize="18">{config.unit}</text>
    <text x="150" y="226" textAnchor="middle" fill={muted} fontSize="12">{config.label}</text>
  </svg>;
}
function Steering({ state, actual, target }: { state: State; actual: number | undefined; target: number | undefined }): ReactElement {
  const rotation = (value: number) => value * state.rotationScale * (state.invertRotation ? -1 : 1);
  return <div style={{ display: "flex", flexDirection: "column", gap: 8, minHeight: 0 }}>
    <svg viewBox="0 0 300 240" role="img" aria-label="Steering: top marker actual, wheel target" style={{ width: "100%", maxHeight: 240, minHeight: 110 }}>
      <circle cx="150" cy="112" r="87" fill="none" stroke={track} strokeWidth="2" />
      <line x1="150" x2="150" y1="12" y2="24" stroke={muted} strokeWidth="1" />
      <g transform={`rotate(${target == undefined ? 0 : rotation(target)},150,112)`} stroke={target == undefined ? track : "#ececf0"} strokeWidth="6" fill="none">
        <circle cx="150" cy="112" r="76" />
        <path d="M 77 92 L 134 110 M 223 92 L 166 110 M 150 129 L 150 188" />
        <circle cx="150" cy="112" r="17" fill="transparent" />
      </g>
      {actual != undefined && <line x1="150" x2="150" y1="22" y2="39" transform={`rotate(${rotation(actual)},150,112)`} stroke={state.primary.color} strokeWidth="6" />}
      <text x="150" y="231" textAnchor="middle" fill="#ececf0" fontSize="27">{formatted(actual, state.primary)}{state.primary.unit}</text>
    </svg>
    <Scale config={state.primary} value={actual} target={target} />
    <div style={{ display: "flex", justifyContent: "space-between", fontSize: 12 }}>
      <span style={{ color: state.primary.color }}>{state.primary.label}: {formatted(actual, state.primary)}{state.primary.unit}</span>
      <span>{state.secondary.label}: {formatted(target, state.secondary)}{state.secondary.unit}</span>
    </div>
  </div>;
}
function GaugePanel({ context, kind }: { context: PanelExtensionContext; kind: Kind }): ReactElement {
  const [state, setState] = useState(() => initialState(context, kind));
  const [readings, setReadings] = useState<Reading[]>([]);
  const [now, setNow] = useState(0);
  const twoSources = kind === "electrical" || kind === "steering";
  const actionHandler = useCallback((action: SettingsTreeAction) => {
    if (action.action === "update" && action.payload.path[0] === "general") {
      handleSettingsAction<State>({ ...action, payload: { ...action.payload, path: action.payload.path.slice(1) } }, setState);
    } else {
      handleSettingsAction<State>(action, setState);
    }
  }, []);
  useEffect(() => {
    context.saveState(state);
    context.setDefaultPanelTitle(kind === "rpm" ? "RPM" : kind === "temperature" ? "Temperature" : kind === "thermometer" ? "Thermometer" : kind === "steering" ? "Steering" : "Current & Voltage");
    updateSettingsEditor(context, actionHandler, {
      primary: { label: kind === "steering" ? "Actual angle" : kind === "electrical" ? "Current" : "Value", icon: "Settings", fields: sourceSettings(state.primary) },
      ...(twoSources ? { secondary: { label: kind === "steering" ? "Target angle" : "Voltage", icon: "Settings" as const, fields: sourceSettings(state.secondary) } } : {}),
      general: { label: "Display", icon: "Settings", fields: {
        timeoutMs: settingsNumber("Stale after (ms)", state.timeoutMs, { min: 0, help: "0 disables timeout. Uses data-source time, including during playback." }),
        ...(kind === "temperature" ? { vertical: settingsBoolean("Vertical scale", state.vertical) } : {}),
        ...(kind === "steering" ? {
          rotationScale: settingsNumber("Wheel rotation scale", state.rotationScale, { help: "Angles must be degrees after value scaling. Only affects wheel and marker rotation." }),
          invertRotation: settingsBoolean("Invert wheel rotation", state.invertRotation),
        } : {}),
      } },
    });
  }, [context, state, actionHandler, kind, twoSources]);
  useEffect(() => {
    const sources = twoSources ? [state.primary, state.secondary] : [state.primary];
    let current: Reading[] = [];
    setReadings([]);
    context.watch("currentFrame");
    context.watch("currentTime");
    context.watch("didSeek");
    context.watch("topics");
    context.subscribe([...new Set(sources.map((s) => s.topic).filter(Boolean))].map((topic) => ({ topic, sampling: { mode: "latest-per-render-tick" as const } })));
    context.onRender = (renderState, done) => {
      if (renderState.didSeek) current = [];
      let changed = Boolean(renderState.didSeek);
      if (renderState.topics?.length === 0) { current = []; changed = true; }
      for (const event of renderState.currentFrame ?? []) {
        sources.forEach((s, i) => {
          if (event.topic !== s.topic) return;
          const raw = numericField(event.message, s.field);
          const value = raw == undefined ? undefined : raw * s.scale;
          current[i] = value != undefined && Number.isFinite(value) ? { value, time: event.receiveTime.sec * 1000 + event.receiveTime.nsec / 1e6 } : undefined;
          changed = true;
        });
      }
      if (changed) setReadings([...current]);
      if (renderState.currentTime) setNow(renderState.currentTime.sec * 1000 + renderState.currentTime.nsec / 1e6);
      done();
    };
    return () => { context.onRender = undefined; context.unsubscribeAll(); };
  }, [context, state.primary.topic, state.primary.field, state.primary.scale, state.secondary.topic, state.secondary.field, state.secondary.scale, twoSources]);
  const value = (i: number): number | undefined => {
    const reading = readings[i];
    return reading && (state.timeoutMs <= 0 || Math.max(0, now - reading.time) <= state.timeoutMs) ? reading.value : undefined;
  };
  const primary = value(0), secondary = value(1);
  return <div style={surface}>
    {kind === "steering" ? <Steering state={state} actual={primary} target={secondary} /> :
      kind === "electrical" ? <><Linear config={state.primary} value={primary} /><div style={{ height: 1, background: track, margin: "4px 0" }} /><Linear config={state.secondary} value={secondary} /></> :
      !validRange(state.primary) ? <span style={{ color: "#efb84b" }}>Maximum must exceed minimum</span> :
      kind === "rpm" ? <Ring config={state.primary} value={primary} /> :
      kind === "thermometer" ? <Thermometer config={state.primary} value={primary} /> :
      state.vertical ? <VerticalTemperature config={state.primary} value={primary} /> : <Linear config={state.primary} value={primary} />}
  </div>;
}
function init(context: PanelExtensionContext, kind: Kind): () => void {
  const root = createRoot(context.panelElement);
  root.render(<GaugePanel context={context} kind={kind} />);
  return () => root.unmount();
}
export const initRpmPanel = (context: PanelExtensionContext): (() => void) => init(context, "rpm");
export const initTemperaturePanel = (context: PanelExtensionContext): (() => void) => init(context, "temperature");
export const initSteeringPanel = (context: PanelExtensionContext): (() => void) => init(context, "steering");
export const initElectricalPanel = (context: PanelExtensionContext): (() => void) => init(context, "electrical");

export const initThermometerPanel = (context: PanelExtensionContext): (() => void) => init(context, "thermometer");
