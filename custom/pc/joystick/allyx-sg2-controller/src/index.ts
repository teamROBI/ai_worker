import { ExtensionContext } from "@foxglove/extension";

import { initControlPanel } from "./ExamplePanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Ally SG2 Controller", initPanel: initControlPanel });
}
