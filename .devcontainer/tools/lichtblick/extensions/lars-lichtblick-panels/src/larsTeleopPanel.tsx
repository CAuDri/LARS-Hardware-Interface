import { PanelExtensionContext, SettingsTreeAction } from "@lichtblick/suite";
import { ReactElement, useCallback, useEffect, useMemo, useState } from "react";
import { createRoot } from "react-dom/client";

import {
  Button,
  advertiseTopic,
  boolFrom,
  clamp,
  handleSettingsAction,
  joinTopic,
  labelStyle,
  numberFrom,
  panelStyle,
  publishMessage,
  rosHeader,
  sectionStyle,
  settingsNumber,
  settingsString,
  stringFrom,
  updateSettingsEditor,
} from "./panelUtils";

type TeleopState = {
  namespace: string;
  publishRateHz: number;
  steering: {
    topicSuffix: string;
    schemaName: string;
    minDeg: number;
    maxDeg: number;
    stepDeg: number;
    valueDeg: number;
    enabled: boolean;
  };
  motor: {
    mode: "rpm" | "current";
    rpmTopicSuffix: string;
    rpmSchemaName: string;
    currentTopicSuffix: string;
    currentSchemaName: string;
    minRpm: number;
    maxRpm: number;
    rpmStep: number;
    rpmValue: number;
    rpmEnabled: boolean;
    minCurrentA: number;
    maxCurrentA: number;
    currentStepA: number;
    currentValueA: number;
    currentEnabled: boolean;
  };
};

const defaultState: TeleopState = {
  namespace: "/hardware",
  publishRateHz: 20,
  steering: {
    topicSuffix: "command/steering_angle",
    schemaName: "lars_msgs/msg/SteeringAngleCommand",
    minDeg: -30,
    maxDeg: 30,
    stepDeg: 1,
    valueDeg: 0,
    enabled: false,
  },
  motor: {
    mode: "rpm",
    rpmTopicSuffix: "command/motor_rpm",
    rpmSchemaName: "lars_msgs/msg/MotorRpmCommand",
    currentTopicSuffix: "command/motor_current",
    currentSchemaName: "lars_msgs/msg/MotorCurrentCommand",
    minRpm: -10000,
    maxRpm: 10000,
    rpmStep: 100,
    rpmValue: 0,
    rpmEnabled: false,
    minCurrentA: -50,
    maxCurrentA: 50,
    currentStepA: 0.5,
    currentValueA: 0,
    currentEnabled: false,
  },
};

function initialState(context: PanelExtensionContext): TeleopState {
  const partial = context.initialState as Partial<TeleopState>;
  const steering: Partial<TeleopState["steering"]> = partial.steering ?? {};
  const motor: Partial<TeleopState["motor"]> = partial.motor ?? {};

  return {
    namespace: stringFrom(partial.namespace, defaultState.namespace),
    publishRateHz: numberFrom(partial.publishRateHz, defaultState.publishRateHz),
    steering: {
      topicSuffix: stringFrom(steering.topicSuffix, defaultState.steering.topicSuffix),
      schemaName: stringFrom(steering.schemaName, defaultState.steering.schemaName),
      minDeg: numberFrom(steering.minDeg, defaultState.steering.minDeg),
      maxDeg: numberFrom(steering.maxDeg, defaultState.steering.maxDeg),
      stepDeg: numberFrom(steering.stepDeg, defaultState.steering.stepDeg),
      valueDeg: numberFrom(steering.valueDeg, defaultState.steering.valueDeg),
      enabled: boolFrom(steering.enabled, defaultState.steering.enabled),
    },
    motor: {
      mode: motor.mode === "current" ? "current" : "rpm",
      rpmTopicSuffix: stringFrom(motor.rpmTopicSuffix, defaultState.motor.rpmTopicSuffix),
      rpmSchemaName: stringFrom(motor.rpmSchemaName, defaultState.motor.rpmSchemaName),
      currentTopicSuffix: stringFrom(
        motor.currentTopicSuffix,
        defaultState.motor.currentTopicSuffix,
      ),
      currentSchemaName: stringFrom(motor.currentSchemaName, defaultState.motor.currentSchemaName),
      minRpm: numberFrom(motor.minRpm, defaultState.motor.minRpm),
      maxRpm: numberFrom(motor.maxRpm, defaultState.motor.maxRpm),
      rpmStep: numberFrom(motor.rpmStep, defaultState.motor.rpmStep),
      rpmValue: numberFrom(motor.rpmValue, defaultState.motor.rpmValue),
      rpmEnabled: boolFrom(motor.rpmEnabled, defaultState.motor.rpmEnabled),
      minCurrentA: numberFrom(motor.minCurrentA, defaultState.motor.minCurrentA),
      maxCurrentA: numberFrom(motor.maxCurrentA, defaultState.motor.maxCurrentA),
      currentStepA: numberFrom(motor.currentStepA, defaultState.motor.currentStepA),
      currentValueA: numberFrom(motor.currentValueA, defaultState.motor.currentValueA),
      currentEnabled: boolFrom(motor.currentEnabled, defaultState.motor.currentEnabled),
    },
  };
}

function publishSteering(context: PanelExtensionContext, topic: string, valueDeg: number): boolean {
  return publishMessage(context, topic, { header: rosHeader(), angle: valueDeg });
}

function publishRpm(context: PanelExtensionContext, topic: string, rpm: number): boolean {
  return publishMessage(context, topic, { header: rosHeader(), rpm: Math.trunc(rpm) });
}

function publishCurrent(context: PanelExtensionContext, topic: string, current: number): boolean {
  return publishMessage(context, topic, { header: rosHeader(), current });
}

function ControlRow({
  disabled,
  enabledLabel = "Override",
  label,
  max,
  min,
  onEnabledChange,
  onReset,
  onValueChange,
  step,
  unit,
  value,
  enabled,
}: {
  disabled?: boolean;
  label: string;
  max: number;
  min: number;
  onEnabledChange: (enabled: boolean) => void;
  onReset: () => void;
  onValueChange: (value: number) => void;
  step: number;
  unit: string;
  value: number;
  enabled: boolean;
  enabledLabel?: string;
}): ReactElement {
  const clampedValue = clamp(value, min, max);
  const inputDisabled = disabled === true || !enabled;
  return (
    <div style={{ display: "grid", gap: 6 }}>
      <div style={{ alignItems: "center", display: "flex", gap: 8, justifyContent: "space-between" }}>
        <label style={{ alignItems: "center", display: "flex", gap: 8 }}>
          <input
            checked={enabled}
            disabled={disabled}
            onChange={(event) => onEnabledChange(event.target.checked)}
            type="checkbox"
          />
          <span style={{ fontWeight: 700 }}>{enabledLabel}</span>
        </label>
        <span style={{ color: "#c7cbd4", fontWeight: 700 }}>{label}</span>
      </div>
      <div style={{ alignItems: "center", display: "grid", gap: 8, gridTemplateColumns: "1fr 90px auto" }}>
        <input
          disabled={inputDisabled}
          max={max}
          min={min}
          onChange={(event) => onValueChange(Number(event.target.value))}
          step={step}
          style={{
            filter: inputDisabled ? "grayscale(1)" : undefined,
            opacity: inputDisabled ? 0.45 : 1,
          }}
          type="range"
          value={clampedValue}
        />
        <input
          disabled={inputDisabled}
          max={max}
          min={min}
          onChange={(event) => onValueChange(Number(event.target.value))}
          step={step}
          style={{
            background: "#101318",
            border: "1px solid #3b4250",
            borderRadius: 4,
            boxSizing: "border-box",
            color: inputDisabled ? "#8b929f" : "#f3f4f6",
            font: "inherit",
            minHeight: 30,
            opacity: inputDisabled ? 0.65 : 1,
            padding: "4px 6px",
            width: "100%",
          }}
          type="number"
          value={clampedValue}
        />
        <Button color="#394150" disabled={disabled} onClick={onReset}>
          Reset
        </Button>
      </div>
      <div style={{ color: "#aeb4c0", fontSize: 12 }}>
        {min} to {max} {unit}
      </div>
    </div>
  );
}

function LarsTeleopPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [state, setState] = useState<TeleopState>(() => initialState(context));
  const [lastPublish, setLastPublish] = useState("Waiting");

  const steeringTopic = useMemo(
    () => joinTopic(state.namespace, state.steering.topicSuffix),
    [state.namespace, state.steering.topicSuffix],
  );
  const rpmTopic = useMemo(
    () => joinTopic(state.namespace, state.motor.rpmTopicSuffix),
    [state.namespace, state.motor.rpmTopicSuffix],
  );
  const currentTopic = useMemo(
    () => joinTopic(state.namespace, state.motor.currentTopicSuffix),
    [state.namespace, state.motor.currentTopicSuffix],
  );
  const motorMode = state.motor.mode;
  const motorControl = motorMode === "rpm"
    ? {
        enabled: state.motor.rpmEnabled,
        label: "Motor speed",
        max: state.motor.maxRpm,
        min: state.motor.minRpm,
        step: state.motor.rpmStep,
        unit: "rpm",
        value: state.motor.rpmValue,
      }
    : {
        enabled: state.motor.currentEnabled,
        label: "Motor current",
        max: state.motor.maxCurrentA,
        min: state.motor.minCurrentA,
        step: state.motor.currentStepA,
        unit: "A",
        value: state.motor.currentValueA,
      };

  const setTeleopState = useCallback((updater: (oldState: TeleopState) => TeleopState) => {
    setState((oldState) => {
      const next = updater(oldState);
      return {
        ...next,
        steering: {
          ...next.steering,
          valueDeg: clamp(next.steering.valueDeg, next.steering.minDeg, next.steering.maxDeg),
        },
        motor: {
          ...next.motor,
          mode: next.motor.mode === "current" ? "current" : "rpm",
          rpmValue: clamp(next.motor.rpmValue, next.motor.minRpm, next.motor.maxRpm),
          currentValueA: clamp(
            next.motor.currentValueA,
            next.motor.minCurrentA,
            next.motor.maxCurrentA,
          ),
        },
      };
    });
  }, []);

  const actionHandler = useCallback(
    (action: SettingsTreeAction) => {
      handleSettingsAction<TeleopState>(action, setTeleopState);
    },
    [setTeleopState],
  );

  useEffect(() => {
    context.saveState(state);
    context.setDefaultPanelTitle("LARS Teleop");
    updateSettingsEditor(context, actionHandler, {
      general: {
        label: "General",
        icon: "Settings",
        fields: {
          namespace: settingsString("Hardware namespace", state.namespace),
          publishRateHz: settingsNumber("Publish rate", state.publishRateHz, {
            min: 1,
            max: 100,
            step: 1,
            precision: 0,
          }),
        },
      },
      steering: {
        label: "Steering",
        icon: "Move",
        fields: {
          topicSuffix: settingsString("Topic suffix", state.steering.topicSuffix),
          schemaName: settingsString("Schema", state.steering.schemaName),
          minDeg: settingsNumber("Min angle", state.steering.minDeg, { step: 1 }),
          maxDeg: settingsNumber("Max angle", state.steering.maxDeg, { step: 1 }),
          stepDeg: settingsNumber("Step", state.steering.stepDeg, { min: 0.1, step: 0.1 }),
        },
      },
      motor: {
        label: "Motor",
        icon: "PrecisionManufacturing",
        fields: {
          rpmTopicSuffix: settingsString("RPM topic suffix", state.motor.rpmTopicSuffix),
          rpmSchemaName: settingsString("RPM schema", state.motor.rpmSchemaName),
          minRpm: settingsNumber("Min RPM", state.motor.minRpm, { step: 100, precision: 0 }),
          maxRpm: settingsNumber("Max RPM", state.motor.maxRpm, { step: 100, precision: 0 }),
          rpmStep: settingsNumber("RPM step", state.motor.rpmStep, { min: 1, step: 10 }),
          currentTopicSuffix: settingsString("Current topic suffix", state.motor.currentTopicSuffix),
          currentSchemaName: settingsString("Current schema", state.motor.currentSchemaName),
          minCurrentA: settingsNumber("Min current", state.motor.minCurrentA, { step: 1 }),
          maxCurrentA: settingsNumber("Max current", state.motor.maxCurrentA, { step: 1 }),
          currentStepA: settingsNumber("Current step", state.motor.currentStepA, {
            min: 0.1,
            step: 0.1,
          }),
        },
      },
    });
  }, [actionHandler, context, state]);

  useEffect(() => {
    advertiseTopic(context, steeringTopic, state.steering.schemaName);
    advertiseTopic(context, rpmTopic, state.motor.rpmSchemaName);
    advertiseTopic(context, currentTopic, state.motor.currentSchemaName);
    return () => {
      context.unadvertise?.(steeringTopic);
      context.unadvertise?.(rpmTopic);
      context.unadvertise?.(currentTopic);
    };
  }, [
    context,
    currentTopic,
    rpmTopic,
    state.motor.currentSchemaName,
    state.motor.rpmSchemaName,
    state.steering.schemaName,
    steeringTopic,
  ]);

  useEffect(() => {
    const active =
      state.steering.enabled || state.motor.rpmEnabled || state.motor.currentEnabled;
    if (!active || context.publish == undefined) {
      return;
    }

    const periodMs = Math.max(10, Math.round(1000 / clamp(state.publishRateHz, 1, 100)));
    const interval = window.setInterval(() => {
      const sent: string[] = [];
      if (state.steering.enabled && publishSteering(context, steeringTopic, state.steering.valueDeg)) {
        sent.push(`steering ${state.steering.valueDeg.toFixed(1)} deg`);
      }
      if (state.motor.rpmEnabled && publishRpm(context, rpmTopic, state.motor.rpmValue)) {
        sent.push(`rpm ${Math.trunc(state.motor.rpmValue)}`);
      }
      if (
        state.motor.currentEnabled &&
        publishCurrent(context, currentTopic, state.motor.currentValueA)
      ) {
        sent.push(`current ${state.motor.currentValueA.toFixed(1)} A`);
      }
      setLastPublish(sent.length > 0 ? sent.join(", ") : "No active overrides");
    }, periodMs);

    return () => window.clearInterval(interval);
  }, [context, currentTopic, rpmTopic, state, steeringTopic]);

  const publishAvailable = context.publish != undefined;
  const stopOverride = () => {
    if (state.steering.enabled) {
      publishSteering(context, steeringTopic, 0);
    }
    if (state.motor.currentEnabled) {
      publishCurrent(context, currentTopic, 0);
    } else {
      publishRpm(context, rpmTopic, 0);
    }
    setLastPublish("stop override: steering 0, motor 0, overrides disabled");
    setTeleopState((oldState) => ({
      ...oldState,
      steering: {
        ...oldState.steering,
        valueDeg: 0,
        enabled: false,
      },
      motor: {
        ...oldState.motor,
        rpmValue: 0,
        currentValueA: 0,
        rpmEnabled: false,
        currentEnabled: false,
      },
    }));
  };

  return (
    <div style={panelStyle}>
      {!publishAvailable && (
        <div style={{ color: "#ffb4ab", fontWeight: 700 }}>
          This connection does not support client publishing.
        </div>
      )}
      <section style={sectionStyle}>
        <div style={labelStyle}>Steering</div>
        <ControlRow
          disabled={!publishAvailable}
          enabled={state.steering.enabled}
          label="Steering override"
          max={state.steering.maxDeg}
          min={state.steering.minDeg}
          onEnabledChange={(enabled) =>
            setTeleopState((oldState) => ({
              ...oldState,
              steering: { ...oldState.steering, enabled },
            }))
          }
          onReset={() =>
            setTeleopState((oldState) => ({
              ...oldState,
              steering: { ...oldState.steering, valueDeg: 0 },
            }))
          }
          onValueChange={(valueDeg) =>
            setTeleopState((oldState) => ({
              ...oldState,
              steering: { ...oldState.steering, valueDeg },
            }))
          }
          step={state.steering.stepDeg}
          unit="deg"
          value={state.steering.valueDeg}
        />
      </section>
      <section style={sectionStyle}>
        <div style={{ alignItems: "center", display: "flex", gap: 8, justifyContent: "space-between" }}>
          <div style={labelStyle}>Motor</div>
          <select
            disabled={!publishAvailable}
            onChange={(event) => {
              const mode = event.target.value === "current" ? "current" : "rpm";
              setTeleopState((oldState) => {
                const keepEnabled = oldState.motor.rpmEnabled || oldState.motor.currentEnabled;
                return {
                  ...oldState,
                  motor: {
                    ...oldState.motor,
                    mode,
                    rpmEnabled: mode === "rpm" ? keepEnabled : false,
                    currentEnabled: mode === "current" ? keepEnabled : false,
                  },
                };
              });
            }}
            style={{
              background: "#101318",
              border: "1px solid #3b4250",
              borderRadius: 4,
              color: "#f3f4f6",
              font: "inherit",
              minHeight: 30,
              padding: "4px 6px",
            }}
            value={motorMode}
          >
            <option value="rpm">RPM</option>
            <option value="current">Current</option>
          </select>
        </div>
        <ControlRow
          disabled={!publishAvailable}
          enabled={motorControl.enabled}
          enabledLabel="Motor override"
          label={motorControl.label}
          max={motorControl.max}
          min={motorControl.min}
          onEnabledChange={(enabled) =>
            setTeleopState((oldState) => ({
              ...oldState,
              motor: {
                ...oldState.motor,
                rpmEnabled: oldState.motor.mode === "rpm" ? enabled : false,
                currentEnabled: oldState.motor.mode === "current" ? enabled : false,
              },
            }))
          }
          onReset={() =>
            setTeleopState((oldState) => ({
              ...oldState,
              motor:
                oldState.motor.mode === "rpm"
                  ? { ...oldState.motor, rpmValue: 0 }
                  : { ...oldState.motor, currentValueA: 0 },
            }))
          }
          onValueChange={(value) =>
            setTeleopState((oldState) => ({
              ...oldState,
              motor:
                oldState.motor.mode === "rpm"
                  ? { ...oldState.motor, rpmValue: value }
                  : { ...oldState.motor, currentValueA: value },
            }))
          }
          step={motorControl.step}
          unit={motorControl.unit}
          value={motorControl.value}
        />
      </section>
      <section style={sectionStyle}>
        <div style={labelStyle}>Override Actions</div>
        <Button
          color="#a11d2a"
          disabled={
            !publishAvailable ||
            (!state.steering.enabled && !state.motor.rpmEnabled && !state.motor.currentEnabled)
          }
          onClick={stopOverride}
        >
          Stop override
        </Button>
        <div style={{ color: "#aeb4c0", fontSize: 12 }}>
          Publishes final zero steering and motor commands, then disables active overrides.
        </div>
      </section>
      <section style={{ ...sectionStyle, gap: 4 }}>
        <div style={labelStyle}>Status</div>
        <div>{lastPublish}</div>
        <div style={{ color: "#aeb4c0", fontSize: 12 }}>
          {state.publishRateHz} Hz to {state.namespace}
        </div>
      </section>
    </div>
  );
}

export function initLarsTeleopPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<LarsTeleopPanel context={context} />);
  return () => root.unmount();
}
