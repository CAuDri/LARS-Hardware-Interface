import { ExtensionContext } from "@lichtblick/suite";

import { initHeaderPanel } from "./HeaderPanel";
import { initLightsPanel } from "./LightsPanel";
import { initTeleopPanel } from "./TeleopPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Header", initPanel: initHeaderPanel });
  extensionContext.registerPanel({ name: "Teleop", initPanel: initTeleopPanel });
  extensionContext.registerPanel({ name: "Lights", initPanel: initLightsPanel });
}
