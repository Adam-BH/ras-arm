import { useArm, type Mode } from '../store/useArm';
import { SerialLink } from '../link/SerialLink';

export function TopBar() {
  const { mode, setMode, connected, status, version, tel, connect, disconnect, reconnect, link } = useArm();
  const usbOk = SerialLink.supported();
  const lost = !connected && !!link && mode === 'usb';
  return (
    <header className="topbar">
      <strong className="brand">RAS Arm</strong>
      <div className="seg" role="group" aria-label="Connection mode">
        {(['virtual', 'usb'] as Mode[]).map(m => (
          <button key={m} className={mode === m ? 'on' : ''} onClick={() => setMode(m)} disabled={m === 'usb' && !usbOk}
            title={m === 'usb' && !usbOk ? 'Web Serial needs Chrome or Edge' : ''}>
            {m === 'virtual' ? 'Virtual' : 'USB'}
          </button>
        ))}
      </div>
      {connected ? <button onClick={disconnect}>Disconnect</button>
        : lost ? <button className="primary" onClick={reconnect}>Reconnect</button>
        : <button className="primary" onClick={connect}>Connect</button>}
      <span className={`dot ${connected ? 'ok' : ''}`} /> <span className="muted small">{status}</span>
      <span className="spacer" />
      {version && <span className="badge">fw {version}</span>}
      <span className={`badge state s-${tel?.state ?? 'none'}`}>{tel?.state ?? '—'}</span>
      {tel && tel.err !== 0 && <span className="badge bad">{tel.errName}</span>}
      <span className={`badge ${tel?.estop ? 'bad' : 'ok'}`}>{tel?.estop ? 'E-STOP' : 'power OK'}</span>
    </header>
  );
}
