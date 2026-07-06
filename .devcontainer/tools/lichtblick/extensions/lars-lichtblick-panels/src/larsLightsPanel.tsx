import { PanelExtensionContext, SettingsTreeAction } from "@lichtblick/suite";
import { ReactElement, useCallback, useEffect, useMemo, useState } from "react";
import { createRoot } from "react-dom/client";

import {
  Button,
  advertiseTopic,
  handleSettingsAction,
  joinTopic,
  labelStyle,
  panelStyle,
  publishMessage,
  sectionStyle,
  settingsString,
  stringFrom,
  updateSettingsEditor,
} from "./panelUtils";

type LightCommand = {
  label: string;
  payload: string;
};

type LightsState = {
  namespace: string;
  topicSuffix: string;
  schemaName: string;
  commands: {
    left: LightCommand;
    right: LightCommand;
    hazard: LightCommand;
    off: LightCommand;
  };
};

const defaultState: LightsState = {
  namespace: "/hardware",
  topicSuffix: "command/lights",
  schemaName: "std_msgs/msg/String",
  commands: {
    left: { label: "Left blinker", payload: "left_blinker" },
    right: { label: "Right blinker", payload: "right_blinker" },
    hazard: { label: "Hazard", payload: "hazard" },
    off: { label: "Lights off", payload: "off" },
  },
};

function initialState(context: PanelExtensionContext): LightsState {
  const partial = context.initialState as Partial<LightsState>;
  const commands: Partial<LightsState["commands"]> = partial.commands ?? {};
  return {
    namespace: stringFrom(partial.namespace, defaultState.namespace),
    topicSuffix: stringFrom(partial.topicSuffix, defaultState.topicSuffix),
    schemaName: stringFrom(partial.schemaName, defaultState.schemaName),
    commands: {
      left: {
        label: stringFrom(commands.left?.label, defaultState.commands.left.label),
        payload: stringFrom(commands.left?.payload, defaultState.commands.left.payload),
      },
      right: {
        label: stringFrom(commands.right?.label, defaultState.commands.right.label),
        payload: stringFrom(commands.right?.payload, defaultState.commands.right.payload),
      },
      hazard: {
        label: stringFrom(commands.hazard?.label, defaultState.commands.hazard.label),
        payload: stringFrom(commands.hazard?.payload, defaultState.commands.hazard.payload),
      },
      off: {
        label: stringFrom(commands.off?.label, defaultState.commands.off.label),
        payload: stringFrom(commands.off?.payload, defaultState.commands.off.payload),
      },
    },
  };
}

function LarsLightsPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [state, setState] = useState<LightsState>(() => initialState(context));
  const [lastCommand, setLastCommand] = useState("Waiting");
  const topic = useMemo(
    () => joinTopic(state.namespace, state.topicSuffix),
    [state.namespace, state.topicSuffix],
  );

  const actionHandler = useCallback(
    (action: SettingsTreeAction) => {
      handleSettingsAction<LightsState>(action, setState);
    },
    [],
  );

  useEffect(() => {
    context.saveState(state);
    context.setDefaultPanelTitle("LARS Lights");
    updateSettingsEditor(context, actionHandler, {
      topic: {
        label: "Topic",
        icon: "Settings",
        fields: {
          namespace: settingsString("Hardware namespace", state.namespace),
          topicSuffix: settingsString("Topic suffix", state.topicSuffix),
          schemaName: settingsString("Schema", state.schemaName),
        },
      },
      commands: {
        label: "Commands",
        icon: "Flag",
        children: {
          left: {
            label: "Left",
            fields: {
              label: settingsString("Label", state.commands.left.label),
              payload: settingsString("Payload", state.commands.left.payload),
            },
          },
          right: {
            label: "Right",
            fields: {
              label: settingsString("Label", state.commands.right.label),
              payload: settingsString("Payload", state.commands.right.payload),
            },
          },
          hazard: {
            label: "Hazard",
            fields: {
              label: settingsString("Label", state.commands.hazard.label),
              payload: settingsString("Payload", state.commands.hazard.payload),
            },
          },
          off: {
            label: "Off",
            fields: {
              label: settingsString("Label", state.commands.off.label),
              payload: settingsString("Payload", state.commands.off.payload),
            },
          },
        },
      },
    });
  }, [actionHandler, context, state]);

  useEffect(() => {
    advertiseTopic(context, topic, state.schemaName);
    return () => context.unadvertise?.(topic);
  }, [context, state.schemaName, topic]);

  const send = (command: LightCommand) => {
    const ok = publishMessage(context, topic, { data: command.payload });
    setLastCommand(ok ? `${command.label}: ${command.payload}` : "Publishing is not supported");
  };

  const publishAvailable = context.publish != undefined;
  const commands = [
    state.commands.left,
    state.commands.right,
    state.commands.hazard,
    state.commands.off,
  ];

  return (
    <div style={panelStyle}>
      {!publishAvailable && (
        <div style={{ color: "#ffb4ab", fontWeight: 700 }}>
          This connection does not support client publishing.
        </div>
      )}
      <section style={sectionStyle}>
        <div style={labelStyle}>Light Commands</div>
        <div style={{ display: "grid", gap: 8, gridTemplateColumns: "repeat(2, minmax(0, 1fr))" }}>
          {commands.map((command) => (
            <Button
              color={command === state.commands.hazard ? "#8b5e15" : "#394150"}
              disabled={!publishAvailable}
              key={command.label}
              onClick={() => send(command)}
              title={command.payload}
            >
              {command.label}
            </Button>
          ))}
        </div>
      </section>
      <section style={{ ...sectionStyle, gap: 4 }}>
        <div style={labelStyle}>Status</div>
        <div>{lastCommand}</div>
        <div style={{ color: "#aeb4c0", fontSize: 12 }}>{topic}</div>
      </section>
    </div>
  );
}

export function initLarsLightsPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<LarsLightsPanel context={context} />);
  return () => root.unmount();
}
