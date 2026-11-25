import type { PanelExtensionContext } from "@foxglove/extension";
import { useCallback, useEffect, useRef, useState, type ReactElement } from "react";
import { createRoot } from "react-dom/client";

type JogJoint = {
  name: string;
  group: string;
  unit: "m" | "rad";
  position: number | null;
  available: boolean;
};

type JogState = {
  publishers_prepared: boolean;
  robot: { ready: boolean; model?: string; reason?: string; generation?: string };
  robot_type: string;
  base_supported: boolean;
  feedback_fresh: boolean;
  feedback_age: number | null;
  joints: JogJoint[];
};

type JogCommand =
  | { kind: "enable" | "stop" }
  | { kind: "base"; x: number; y: number; yaw: number }
  | { kind: "joint"; joint: string; direction: -1 | 1; resolution: "fine" | "normal" | "coarse" };

type SavedPanelState = {
  managerUrl?: string;
  container?: string;
  mode?: "base" | "joint";
  deadzone?: number;
};

const DEFAULT_URL = "ws://192.168.6.2:8081";
const DEFAULT_CONTAINER = "ai_worker";

function makeJogUrl(managerUrl: string, container: string): string {
  let base = managerUrl.trim();
  if (!/^wss?:\/\//i.test(base)) base = `ws://${base}`;
  base = base.replace(/\/+$/, "");
  const url = new URL(`${base}/ws/jog`);
  url.searchParams.set("container", container.trim());
  return url.toString();
}

function adjustedAxis(value: number, deadzone: number): number {
  if (!Number.isFinite(value) || Math.abs(value) <= deadzone) return 0;
  return Math.sign(value) * (Math.abs(value) - deadzone) / (1 - deadzone);
}

function ControlPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const saved = (context.initialState ?? {}) as SavedPanelState;
  const [managerUrl, setManagerUrl] = useState(saved.managerUrl ?? DEFAULT_URL);
  const [container, setContainer] = useState(saved.container ?? DEFAULT_CONTAINER);
  const [mode, setMode] = useState<"base" | "joint">(saved.mode ?? "base");
  const [deadzone, setDeadzone] = useState(saved.deadzone ?? 0.15);
  const [selectedJoint, setSelectedJoint] = useState("");
  const [connectionTarget, setConnectionTarget] = useState({
    managerUrl: saved.managerUrl ?? DEFAULT_URL,
    container: saved.container ?? DEFAULT_CONTAINER,
  });
  const [attempt, setAttempt] = useState(0);
  const [connected, setConnected] = useState(false);
  const [armed, setArmed] = useState(false);
  const [preparing, setPreparing] = useState(false);
  const [state, setState] = useState<JogState | null>(null);
  const [gamepadName, setGamepadName] = useState("No controller detected");
  const [error, setError] = useState<string | null>(null);

  const socketRef = useRef<WebSocket | null>(null);
  const armedRef = useRef(false);
  const pendingEnableRef = useRef(false);
  const activeMotionRef = useRef(false);
  const lastReplyRef = useRef(0);
  const generationRef = useRef<string | undefined>(undefined);
  const enableTimerRef = useRef<number | undefined>(undefined);
  const controlsRef = useRef({ mode, deadzone, selectedJoint });
  const stateRef = useRef(state);

  useEffect(() => {
    controlsRef.current = { mode, deadzone, selectedJoint };
  }, [mode, deadzone, selectedJoint]);

  useEffect(() => {
    stateRef.current = state;
  }, [state]);

  useEffect(() => {
    context.saveState({ managerUrl, container, mode, deadzone });
  }, [context, managerUrl, container, mode, deadzone]);

  const send = useCallback((command: JogCommand): boolean => {
    const socket = socketRef.current;
    if (socket?.readyState !== WebSocket.OPEN) return false;
    socket.send(JSON.stringify(command));
    return true;
  }, []);

  const disarm = useCallback((sendStop = true) => {
    if (sendStop && (armedRef.current || pendingEnableRef.current || activeMotionRef.current)) {
      send({ kind: "stop" });
    }
    armedRef.current = false;
    pendingEnableRef.current = false;
    activeMotionRef.current = false;
    setPreparing(false);
    setArmed(false);
    if (enableTimerRef.current !== undefined) window.clearTimeout(enableTimerRef.current);
  }, [send]);

  useEffect(() => {
    let url: string;
    try {
      url = makeJogUrl(connectionTarget.managerUrl, connectionTarget.container);
    } catch {
      setError("Enter a valid Cyclo Manager WebSocket address and container name.");
      return;
    }

    let disposed = false;
    let socket: WebSocket;
    try {
      socket = new WebSocket(url);
    } catch {
      setError("Could not open the Cyclo Manager connection.");
      return;
    }
    socketRef.current = socket;
    setConnected(false);
    setState(null);
    setError(null);
    setArmed(false);
    armedRef.current = false;
    pendingEnableRef.current = false;
    generationRef.current = undefined;

    socket.onopen = () => {
      if (disposed) return;
      lastReplyRef.current = Date.now();
      setConnected(true);
    };
    socket.onmessage = (event) => {
      if (disposed) return;
      try {
        const message = JSON.parse(String(event.data)) as { state?: JogState; error?: string };
        lastReplyRef.current = Date.now();
        if (message.error) {
          setError(message.error);
          disarm(false);
          socket.close();
          return;
        }
        if (!message.state) return;
        const next = message.state;
        const nextGeneration = next.robot?.generation;
        if (generationRef.current !== undefined && nextGeneration !== generationRef.current) {
          disarm(true);
        }
        generationRef.current = nextGeneration;
        if (!next.robot?.ready) disarm(true);
        if (pendingEnableRef.current && next.publishers_prepared && next.robot?.ready) {
          pendingEnableRef.current = false;
          armedRef.current = true;
          setPreparing(false);
          setArmed(true);
          if (enableTimerRef.current !== undefined) window.clearTimeout(enableTimerRef.current);
        }
        setState(next);
      } catch {
        setError("Cyclo Manager sent an unreadable Jog status. Reconnect to continue.");
        disarm(true);
        socket.close();
      }
    };
    socket.onerror = () => {
      if (!disposed) {
        setError("Cyclo Manager connection failed.");
        disarm(false);
      }
    };
    socket.onclose = () => {
      if (!disposed) {
        setConnected(false);
        disarm(false);
      }
    };

    const feedbackWatch = window.setInterval(() => {
      if (disposed || socket.readyState !== WebSocket.OPEN) return;
      if (Date.now() - lastReplyRef.current > 1000) {
        setError("Jog feedback timed out. Reconnect before moving.");
        disarm(true);
        socket.close();
      }
    }, 250);

    const loseFocus = () => disarm(true);
    const visibility = () => {
      if (document.hidden) disarm(true);
    };
    window.addEventListener("blur", loseFocus);
    window.addEventListener("pagehide", loseFocus);
    document.addEventListener("visibilitychange", visibility);

    return () => {
      disposed = true;
      disarm(true);
      window.clearInterval(feedbackWatch);
      window.removeEventListener("blur", loseFocus);
      window.removeEventListener("pagehide", loseFocus);
      document.removeEventListener("visibilitychange", visibility);
      if (socket.readyState === WebSocket.OPEN) socket.close();
      if (socketRef.current === socket) socketRef.current = null;
    };
  }, [connectionTarget, attempt, disarm]);

  useEffect(() => {
    if (!state?.joints) return;
    const selectedAvailable = state.joints.some((joint) => joint.name === selectedJoint && joint.available);
    if (selectedJoint && !selectedAvailable && activeMotionRef.current) disarm(true);
    if (!selectedAvailable) {
      const firstAvailable = state.joints.find((joint) => joint.available);
      if (firstAvailable && firstAvailable.name !== selectedJoint) setSelectedJoint(firstAvailable.name);
    }
  }, [selectedJoint, state, disarm]);

  useEffect(() => {
    const poll = window.setInterval(() => {
      const pad = Array.from(navigator.getGamepads?.() ?? []).find((candidate) => candidate?.connected);
      const nextName = pad ? `${pad.id}${pad.mapping ? ` · ${pad.mapping}` : ""}` : "No controller detected";
      setGamepadName((current) => current === nextName ? current : nextName);

      if (!pad || document.hidden || !document.hasFocus()) {
        if (armedRef.current || activeMotionRef.current) disarm(true);
        return;
      }

      if (!armedRef.current) return;
      // Xbox/standard Gamepad mapping: LB (button 4) is the motion deadman.
      if (!(pad.buttons[4]?.pressed || (pad.buttons[4]?.value ?? 0) > 0.5)) {
        if (activeMotionRef.current) {
          send({ kind: "stop" });
          activeMotionRef.current = false;
        }
        return;
      }

      const controls = controlsRef.current;
      const jogState = stateRef.current;
      if (controls.mode === "base") {
        if (!jogState?.base_supported) {
          if (activeMotionRef.current) send({ kind: "stop" });
          activeMotionRef.current = false;
          return;
        }
        const leftX = adjustedAxis(pad.axes[0] ?? 0, controls.deadzone);
        const leftY = adjustedAxis(pad.axes[1] ?? 0, controls.deadzone);
        const rightX = adjustedAxis(pad.axes[2] ?? 0, controls.deadzone);
        const x = -leftY * 0.3;
        const y = -leftX * 0.3;
        const yaw = -rightX * 0.6;
        if (Math.max(Math.abs(x), Math.abs(y), Math.abs(yaw)) > 0) {
          send({ kind: "base", x, y, yaw });
          activeMotionRef.current = true;
        } else if (activeMotionRef.current) {
          send({ kind: "stop" });
          activeMotionRef.current = false;
        }
        return;
      }

      const joint = jogState?.joints.find((item) => item.name === controls.selectedJoint && item.available);
      const stick = adjustedAxis(-(pad.axes[1] ?? 0), controls.deadzone);
      if (joint && stick !== 0) {
        const magnitude = Math.abs(stick);
        const resolution = magnitude >= 0.7 ? "coarse" : magnitude >= 0.35 ? "normal" : "fine";
        send({ kind: "joint", joint: joint.name, direction: stick > 0 ? 1 : -1, resolution });
        activeMotionRef.current = true;
      } else if (activeMotionRef.current) {
        send({ kind: "stop" });
        activeMotionRef.current = false;
      }
    }, 100);

    return () => window.clearInterval(poll);
  }, [disarm, send]);

  const connect = () => {
    disarm(true);
    setConnectionTarget({ managerUrl: managerUrl.trim(), container: container.trim() });
    setAttempt((current) => current + 1);
  };

  const toggleJog = () => {
    if (armedRef.current || pendingEnableRef.current) {
      disarm(true);
      return;
    }
    if (!connected || !state?.robot?.ready) return;
    if (!send({ kind: "enable" })) return;
    pendingEnableRef.current = true;
    setPreparing(true);
    setError(null);
    enableTimerRef.current = window.setTimeout(() => {
      if (!pendingEnableRef.current) return;
      disarm(true);
      setError("Jog preparation timed out. Reconnect to continue.");
    }, 5000);
  };

  const changeMode = (next: "base" | "joint") => {
    if (next !== mode) {
      disarm(true);
      setMode(next);
    }
  };

  const changeJoint = (next: string) => {
    if (next !== selectedJoint) disarm(true);
    setSelectedJoint(next);
  };

  const statusColor = !connected ? "#f0b35a" : state?.robot?.ready ? "#54d18b" : "#f07178";
  const selected = state?.joints.find((joint) => joint.name === selectedJoint);

  return (
    <main style={{ color: "#e8edf2", background: "#14191f", minHeight: "100%", padding: 16, fontFamily: "system-ui, sans-serif", boxSizing: "border-box" }}>
      <header style={{ display: "flex", justifyContent: "space-between", alignItems: "center", gap: 12, flexWrap: "wrap", borderBottom: "1px solid #303943", paddingBottom: 12 }}>
        <div>
          <div style={{ color: "#aab5c0", fontSize: 12, letterSpacing: 1.2 }}>ROBOTIS · AI WORKER SG2</div>
          <h2 style={{ margin: "4px 0 0", fontSize: 22 }}>Ally Controller</h2>
        </div>
        <div style={{ color: statusColor, fontWeight: 700, fontSize: 13 }}>{connected ? (state?.robot?.ready ? "ROBOT READY" : "ROBOT NOT READY") : "DISCONNECTED"}</div>
      </header>

      <section style={{ display: "grid", gridTemplateColumns: "minmax(220px, 1fr) minmax(220px, 1fr)", gap: 12, marginTop: 14 }}>
        <label style={{ display: "grid", gap: 5, color: "#aab5c0", fontSize: 12 }}>
          Cyclo Manager WebSocket
          <input value={managerUrl} onFocus={() => disarm(true)} onChange={(event) => setManagerUrl(event.target.value)} spellCheck={false} style={inputStyle} />
        </label>
        <label style={{ display: "grid", gap: 5, color: "#aab5c0", fontSize: 12 }}>
          Robot container
          <input value={container} onFocus={() => disarm(true)} onChange={(event) => setContainer(event.target.value)} spellCheck={false} style={inputStyle} />
        </label>
      </section>

      <section style={{ display: "flex", alignItems: "center", gap: 10, flexWrap: "wrap", marginTop: 10 }}>
        <button onClick={connect} style={buttonStyle}>{connected ? "Reconnect" : "Connect"}</button>
        <span style={{ color: "#aab5c0", fontSize: 12 }}>{gamepadName}</span>
        <span style={{ color: "#71808f", fontSize: 12 }}>Connects to /ws/jog · ROS commands stay on Cyclo Manager</span>
      </section>

      <section style={{ display: "flex", gap: 8, marginTop: 18 }}>
        <button onClick={() => changeMode("base")} aria-pressed={mode === "base"} disabled={connected && state?.base_supported === false} style={modeButtonStyle(mode === "base")}>BASE</button>
        <button onClick={() => changeMode("joint")} aria-pressed={mode === "joint"} style={modeButtonStyle(mode === "joint")}>JOINT</button>
        {mode === "joint" && (
          <select aria-label="Select joint" value={selectedJoint} onChange={(event) => changeJoint(event.target.value)} style={{ ...inputStyle, flex: 1, maxWidth: 360 }}>
            <option value="">Select a joint</option>
            {(state?.joints ?? []).map((joint) => (
              <option key={joint.name} value={joint.name} disabled={!joint.available}>
                {joint.group ? `${joint.group} · ` : ""}{joint.name}{joint.available ? "" : " · unavailable"}
              </option>
            ))}
          </select>
        )}
      </section>

      <section style={{ display: "grid", gridTemplateColumns: "minmax(220px, 1.3fr) minmax(180px, 0.7fr)", gap: 12, marginTop: 12 }}>
        <div style={cardStyle}>
          <div style={{ fontSize: 13, color: "#aab5c0", marginBottom: 8 }}>Controller mapping</div>
          {mode === "base" ? (
            <div style={{ lineHeight: 1.65 }}>
              <b>Hold LB</b> to move · left stick drives forward / sideways · right stick turns.<br />
              Releasing LB or centering the sticks sends stop.
            </div>
          ) : (
            <div style={{ lineHeight: 1.65 }}>
              <b>Hold LB</b> and move the left stick up/down to jog the selected joint.<br />
              Stick distance selects fine, normal, or coarse increments.
            </div>
          )}
          <label style={{ display: "flex", alignItems: "center", gap: 10, marginTop: 12, color: "#aab5c0", fontSize: 12 }}>
            Stick dead zone {Math.round(deadzone * 100)}%
            <input type="range" min="0.05" max="0.35" step="0.01" value={deadzone} onChange={(event) => setDeadzone(Number(event.target.value))} />
          </label>
          {mode === "joint" && selected && <div style={{ color: "#aab5c0", marginTop: 8 }}>Position feedback: {selected.position === null ? "unavailable" : selected.unit === "m" ? `${(selected.position * 1000).toFixed(1)} mm` : `${(selected.position * 180 / Math.PI).toFixed(1)}°`}</div>}
        </div>
        <div style={cardStyle}>
          <div style={{ fontSize: 13, color: "#aab5c0", marginBottom: 8 }}>Jog interlock</div>
          <button onClick={toggleJog} disabled={!connected || !state?.robot?.ready} style={{ ...buttonStyle, width: "100%", background: armed ? "#7c343b" : "#236d4a", opacity: !connected || !state?.robot?.ready ? 0.5 : 1 }}>
            {preparing ? "Preparing…" : armed ? "STOP + DISARM" : "ENABLE JOG"}
          </button>
          <div style={{ marginTop: 8, color: "#aab5c0", fontSize: 12 }}>
            {state?.robot?.ready ? `${state.robot.model ?? state.robot_type} · ${state.feedback_fresh ? "feedback fresh" : "feedback stale"}` : state?.robot?.reason ?? "Connect and confirm robot readiness."}
          </div>
        </div>
      </section>

      {error && <div role="alert" style={{ marginTop: 12, padding: 10, border: "1px solid #8b3940", borderRadius: 6, color: "#ff9ca3", background: "#321e22" }}>{error}</div>}
      <footer style={{ marginTop: 14, color: "#71808f", fontSize: 11, lineHeight: 1.5 }}>
        Gamepad motion is disabled until Jog is explicitly enabled. Focus loss, controller disconnect, feedback timeout, or socket close disarms and requests stop. Keep Foxglove on the robot Wi-Fi.
      </footer>
    </main>
  );
}

const inputStyle = {
  minWidth: 0,
  border: "1px solid #3a4652",
  borderRadius: 5,
  background: "#202830",
  color: "#e8edf2",
  padding: "8px 9px",
};

const buttonStyle = {
  border: "1px solid #536170",
  borderRadius: 5,
  background: "#29333d",
  color: "#f5f7fa",
  padding: "8px 12px",
  cursor: "pointer",
  fontWeight: 700,
};

const cardStyle = {
  border: "1px solid #303943",
  borderRadius: 7,
  padding: 12,
  background: "#1a2027",
  minWidth: 0,
};

function modeButtonStyle(active: boolean) {
  return {
    ...buttonStyle,
    background: active ? "#315d7a" : buttonStyle.background,
    borderColor: active ? "#5ca8d6" : buttonStyle.border,
  };
}

export function initControlPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<ControlPanel context={context} />);
  return () => root.unmount();
}
