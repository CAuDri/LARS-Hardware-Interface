import {
  Immutable,
  MessageEvent,
  PanelExtensionContext,
  RenderState,
  SettingsTreeAction,
} from "@lichtblick/suite";
import { ReactElement, useCallback, useEffect, useMemo, useState } from "react";
import { createRoot } from "react-dom/client";

import {
  Button,
  boolFrom,
  handleSettingsAction,
  joinTopic,
  numberFrom,
  settingsBoolean,
  settingsNumber,
  settingsString,
  stringFrom,
  updateSettingsEditor,
} from "./panelUtils";

type HeaderState = {
  namespace: string;
  heartbeatTopicSuffix: string;
  heartbeatTimeoutMs: number;
  resetServiceSuffix: string;
  emergencyStopServiceSuffix: string;
  serviceRequest: Record<string, unknown>;
  showHeartbeat: boolean;
};

const defaultState: HeaderState = {
  namespace: "/hardware",
  heartbeatTopicSuffix: "heartbeat",
  heartbeatTimeoutMs: 4000,
  resetServiceSuffix: "reset",
  emergencyStopServiceSuffix: "emergency_stop",
  serviceRequest: {},
  showHeartbeat: true,
};

type ResponseState = {
  kind: "idle" | "requesting" | "success" | "error";
  text: string;
};

function initialState(context: PanelExtensionContext): HeaderState {
  const partial = context.initialState as Partial<HeaderState>;
  return {
    namespace: stringFrom(partial.namespace, defaultState.namespace),
    heartbeatTopicSuffix: stringFrom(
      partial.heartbeatTopicSuffix,
      defaultState.heartbeatTopicSuffix,
    ),
    heartbeatTimeoutMs: numberFrom(partial.heartbeatTimeoutMs, defaultState.heartbeatTimeoutMs),
    resetServiceSuffix: stringFrom(partial.resetServiceSuffix, defaultState.resetServiceSuffix),
    emergencyStopServiceSuffix: stringFrom(
      partial.emergencyStopServiceSuffix,
      defaultState.emergencyStopServiceSuffix,
    ),
    serviceRequest:
      partial.serviceRequest != undefined &&
      typeof partial.serviceRequest === "object" &&
      !Array.isArray(partial.serviceRequest)
        ? partial.serviceRequest
        : defaultState.serviceRequest,
    showHeartbeat: boolFrom(partial.showHeartbeat, defaultState.showHeartbeat),
  };
}

function formatResponse(response: unknown): string {
  if (response == undefined) {
    return "Service call completed";
  }
  if (typeof response === "object") {
    const objectResponse = response as Record<string, unknown>;
    const message = objectResponse.message;
    const success = objectResponse.success;
    if (typeof success === "boolean" && typeof message === "string") {
      return `${success ? "OK" : "Failed"}: ${message}`;
    }
  }
  return JSON.stringify(
    response,
    (_key, value) => (typeof value === "bigint" ? value.toString() : value),
    0,
  );
}

function messageIsOnTopic(messageEvent: MessageEvent, topic: string): boolean {
  return messageEvent.topic === topic;
}

function LarsHeaderPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [state, setState] = useState<HeaderState>(() => initialState(context));
  const [response, setResponse] = useState<ResponseState>({
    kind: "idle",
    text: "No service calls yet",
  });
  const [lastHeartbeatMs, setLastHeartbeatMs] = useState<number | undefined>();
  const [nowMs, setNowMs] = useState(() => Date.now());

  const heartbeatTopic = useMemo(
    () => joinTopic(state.namespace, state.heartbeatTopicSuffix),
    [state.namespace, state.heartbeatTopicSuffix],
  );
  const resetService = useMemo(
    () => joinTopic(state.namespace, state.resetServiceSuffix),
    [state.namespace, state.resetServiceSuffix],
  );
  const emergencyStopService = useMemo(
    () => joinTopic(state.namespace, state.emergencyStopServiceSuffix),
    [state.namespace, state.emergencyStopServiceSuffix],
  );

  const actionHandler = useCallback(
    (action: SettingsTreeAction) => {
      handleSettingsAction<HeaderState>(action, setState);
    },
    [],
  );

  useEffect(() => {
    context.saveState(state);
    context.setDefaultPanelTitle("LARS Header");
    updateSettingsEditor(context, actionHandler, {
      general: {
        label: "General",
        icon: "Settings",
        fields: {
          namespace: settingsString("Hardware namespace", state.namespace),
        },
      },
      heartbeat: {
        label: "Heartbeat",
        icon: "Circle",
        fields: {
          showHeartbeat: settingsBoolean("Show heartbeat status", state.showHeartbeat),
          heartbeatTopicSuffix: settingsString("Topic suffix", state.heartbeatTopicSuffix),
          heartbeatTimeoutMs: settingsNumber("Timeout", state.heartbeatTimeoutMs, {
            min: 1000,
            max: 10000,
            step: 500,
            precision: 0,
            help: "Board is shown disconnected after this many milliseconds without messages.",
          }),
        },
      },
      services: {
        label: "Services",
        icon: "Settings",
        fields: {
          resetServiceSuffix: settingsString("Reset service suffix", state.resetServiceSuffix),
          emergencyStopServiceSuffix: settingsString(
            "Emergency stop service suffix",
            state.emergencyStopServiceSuffix,
          ),
        },
      },
    });
  }, [actionHandler, context, state]);

  useEffect(() => {
    context.watch("currentFrame");
    context.watch("services");
    context.subscribe([{ topic: heartbeatTopic, sampling: { mode: "latest-per-render-tick" } }]);

    context.onRender = (renderState: Immutable<RenderState>, done) => {
      if (renderState.currentFrame?.some((messageEvent) => messageIsOnTopic(messageEvent, heartbeatTopic))) {
        setLastHeartbeatMs(Date.now());
      }
      done();
    };

    return () => {
      context.onRender = undefined;
      context.unsubscribeAll();
    };
  }, [context, heartbeatTopic]);

  useEffect(() => {
    const interval = window.setInterval(() => setNowMs(Date.now()), 500);
    return () => window.clearInterval(interval);
  }, []);

  const heartbeatAgeMs = lastHeartbeatMs == undefined ? undefined : nowMs - lastHeartbeatMs;
  const boardConnected =
    heartbeatAgeMs != undefined && heartbeatAgeMs <= Math.max(1000, state.heartbeatTimeoutMs);
  const serviceCallAvailable = context.callService != undefined;

  const callService = async (service: string, label: string) => {
    if (context.callService == undefined) {
      setResponse({ kind: "error", text: "Current data source does not support service calls" });
      return;
    }

    try {
      setResponse({ kind: "requesting", text: `Calling ${label}...` });
      const result = await context.callService(service, state.serviceRequest);
      setResponse({ kind: "success", text: `${label}: ${formatResponse(result)}` });
    } catch (error: unknown) {
      setResponse({
        kind: "error",
        text: `${label}: ${error instanceof Error ? error.message : String(error)}`,
      });
    }
  };

  const confirmAndReset = () => {
    const confirmed = window.confirm(
      "Reset the hardware interface?\n\nThis will perform a hardware reset of the microcontroller.",
    );
    if (confirmed) {
      void callService(resetService, "Reset");
    }
  };

  const statusColor = boardConnected ? "#57d37b" : "#ffb84d";
  const responseColor =
    response.kind === "error" ? "#ffb4ab" : response.kind === "success" ? "#8fd5ff" : "#c7cbd4";

  return (
    <div
      style={{
        alignItems: "center",
        background: "#11151c",
        borderBottom: "1px solid #313846",
        boxSizing: "border-box",
        color: "#ececf0",
        display: "grid",
        font: "13px system-ui, sans-serif",
        gap: 12,
        gridTemplateColumns: "minmax(170px, 1fr) auto auto auto",
        height: "100%",
        minHeight: 0,
        overflow: "hidden",
        padding: "8px 12px",
      }}
    >
      <div style={{ alignItems: "center", display: "flex", gap: 8, minWidth: 0 }}>
        {state.showHeartbeat && (
          <div
            style={{
              alignItems: "center",
              color: statusColor,
              display: "flex",
              fontSize: 12,
              fontWeight: 700,
              gap: 7,
              whiteSpace: "nowrap",
            }}
          >
            <span
              style={{
                background: statusColor,
                borderRadius: "50%",
                boxShadow: `0 0 8px ${statusColor}`,
                display: "inline-block",
                height: 9,
                width: 9,
              }}
            />
            {boardConnected ? "Board connected" : "Board offline"}
          </div>
        )}
      </div>
      <div
        style={{
          color: responseColor,
          maxWidth: 420,
          overflow: "hidden",
          textOverflow: "ellipsis",
          whiteSpace: "nowrap",
        }}
        title={response.text}
      >
        {response.text}
      </div>
      <Button
        color="#394150"
        disabled={!serviceCallAvailable || response.kind === "requesting"}
        onClick={confirmAndReset}
        title={resetService}
      >
        Reset
      </Button>
      <Button
        color="#a11d2a"
        disabled={!serviceCallAvailable || response.kind === "requesting"}
        onClick={() => callService(emergencyStopService, "Emergency Stop")}
        title={emergencyStopService}
      >
        Emergency Stop
      </Button>
    </div>
  );
}

export function initLarsHeaderPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<LarsHeaderPanel context={context} />);
  return () => root.unmount();
}
