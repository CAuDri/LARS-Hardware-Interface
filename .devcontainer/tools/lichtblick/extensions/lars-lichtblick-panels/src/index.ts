import { ExtensionContext } from "@lichtblick/suite";

import { initLarsHeaderPanel } from "./larsHeaderPanel";
import { initLarsLightsPanel } from "./larsLightsPanel";
import { initLarsTeleopPanel } from "./larsTeleopPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "LARS Header", initPanel: initLarsHeaderPanel });
  extensionContext.registerPanel({ name: "LARS Teleop", initPanel: initLarsTeleopPanel });
  extensionContext.registerPanel({ name: "LARS Lights", initPanel: initLarsLightsPanel });
}
