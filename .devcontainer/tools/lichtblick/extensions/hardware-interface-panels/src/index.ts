import { ExtensionContext } from "@lichtblick/suite";

import { initTopicMonitorPanel } from "./TopicMonitorPanel";
import { initHeaderPanel } from "./HeaderPanel";
import { initLightsPanel } from "./LightsPanel";
import { initTeleopPanel } from "./TeleopPanel";

import { initRpmPanel, initTemperaturePanel, initSteeringPanel, initElectricalPanel, initThermometerPanel } from "./GaugePanels";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Topic Monitor", initPanel: initTopicMonitorPanel });
  extensionContext.registerPanel({ name: "Header", initPanel: initHeaderPanel });
  extensionContext.registerPanel({ name: "Teleop", initPanel: initTeleopPanel });
  extensionContext.registerPanel({ name: "Lights", initPanel: initLightsPanel });
  extensionContext.registerPanel({ name: "RPM", initPanel: initRpmPanel });
  extensionContext.registerPanel({ name: "Temperature", initPanel: initTemperaturePanel });
  extensionContext.registerPanel({ name: "Steering", initPanel: initSteeringPanel });
  extensionContext.registerPanel({ name: "Current & Voltage", initPanel: initElectricalPanel });
  extensionContext.registerPanel({ name: "Thermometer", initPanel: initThermometerPanel });
}
