import {
  PanelExtensionContext,
  SettingsTree,
  SettingsTreeAction,
  SettingsTreeField,
} from "@lichtblick/suite";
import { CSSProperties, ReactElement } from "react";

export type RosHeader = {
  stamp: {
    sec: number;
    nanosec: number;
  };
  frame_id: string;
};

export function rosHeader(): RosHeader {
  const nowMs = Date.now();
  const sec = Math.floor(nowMs / 1000);
  return {
    stamp: {
      sec,
      nanosec: (nowMs - sec * 1000) * 1000000,
    },
    frame_id: "",
  };
}

export function joinTopic(namespace: string, suffix: string): string {
  const cleanNamespace = namespace.trim().replace(/\/+$/, "");
  const cleanSuffix = suffix.trim().replace(/^\/+/, "");
  if (cleanNamespace.length === 0 || cleanNamespace === "/") {
    return `/${cleanSuffix}`;
  }
  return `${cleanNamespace}/${cleanSuffix}`;
}

export function clamp(value: number, min: number, max: number): number {
  return Math.min(Math.max(value, min), max);
}

export function numberFrom(value: unknown, fallback: number): number {
  return typeof value === "number" && Number.isFinite(value) ? value : fallback;
}

export function stringFrom(value: unknown, fallback: string): string {
  return typeof value === "string" ? value : fallback;
}

export function boolFrom(value: unknown, fallback: boolean): boolean {
  return typeof value === "boolean" ? value : fallback;
}

export function updateByPath<T>(state: T, path: readonly string[], value: unknown): T {
  if (path.length === 0) {
    return state;
  }

  const root = { ...(state as Record<string, unknown>) };
  let node = root;
  for (let index = 0; index < path.length - 1; index++) {
    const key = path[index];
    if (key == undefined) {
      return state;
    }
    const next = node[key];
    const copy = typeof next === "object" && next != undefined ? { ...next } : {};
    node[key] = copy;
    node = copy as Record<string, unknown>;
  }

  const last = path[path.length - 1];
  if (last != undefined) {
    node[last] = value;
  }
  return root as T;
}

export function handleSettingsAction<T>(
  action: SettingsTreeAction,
  setState: (updater: (state: T) => T) => void,
): void {
  if (action.action !== "update") {
    return;
  }

  const { path, value } = action.payload;
  setState((state) => updateByPath(state, path, value));
}

export function advertiseTopic(
  context: PanelExtensionContext,
  topic: string,
  schemaName: string,
): void {
  if (context.advertise == undefined) {
    return;
  }
  context.advertise(topic, schemaName);
}

export function publishMessage(
  context: PanelExtensionContext,
  topic: string,
  message: unknown,
): boolean {
  if (context.publish == undefined) {
    return false;
  }
  context.publish(topic, message);
  return true;
}

export function settingsNumber(
  label: string,
  value: number,
  options: { min?: number; max?: number; step?: number; precision?: number; help?: string } = {},
): SettingsTreeField {
  return {
    input: "number",
    label,
    value,
    min: options.min,
    max: options.max,
    step: options.step,
    precision: options.precision,
    help: options.help,
  };
}

export function settingsString(label: string, value: string, help?: string): SettingsTreeField {
  return {
    input: "string",
    label,
    value,
    help,
  };
}

export function settingsBoolean(label: string, value: boolean, help?: string): SettingsTreeField {
  return {
    input: "boolean",
    label,
    value,
    help,
  };
}

export function updateSettingsEditor(
  context: PanelExtensionContext,
  actionHandler: (action: SettingsTreeAction) => void,
  nodes: SettingsTree["nodes"],
): void {
  context.updatePanelSettingsEditor({ actionHandler, nodes });
}

const buttonBase: CSSProperties = {
  border: "1px solid #5d6470",
  borderRadius: 4,
  color: "#f4f4f5",
  cursor: "pointer",
  font: "inherit",
  minHeight: 32,
  padding: "6px 10px",
};

export function Button({
  children,
  color = "#2f3540",
  disabled = false,
  onClick,
  title,
}: {
  children: string;
  color?: string;
  disabled?: boolean;
  onClick: () => void;
  title?: string;
}): ReactElement {
  return (
    <button
      disabled={disabled}
      onClick={onClick}
      title={title}
      style={{
        ...buttonBase,
        background: disabled ? "#252932" : color,
        color: disabled ? "#8b929f" : "#f4f4f5",
        cursor: disabled ? "not-allowed" : "pointer",
      }}
    >
      {children}
    </button>
  );
}

export const panelStyle: CSSProperties = {
  boxSizing: "border-box",
  color: "#ececf0",
  display: "flex",
  flexDirection: "column",
  font: "13px system-ui, sans-serif",
  gap: 10,
  height: "100%",
  minHeight: 0,
  overflow: "auto",
  padding: 10,
};

export const sectionStyle: CSSProperties = {
  border: "1px solid #343a46",
  borderRadius: 6,
  display: "flex",
  flexDirection: "column",
  gap: 8,
  padding: 10,
};

export const labelStyle: CSSProperties = {
  color: "#c7cbd4",
  fontSize: 12,
  fontWeight: 700,
  textTransform: "uppercase",
};
