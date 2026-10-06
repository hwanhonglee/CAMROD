// HH_261006 - Preserve the WebKit kiosk's legacy defaults. Only a mounted 3D
// view requests graphics; ordinary browsers need no native bridge or override.
let sequence = 0;
// HH_261006 - Late native acknowledgements from a previous document must not
// match a new page's first lease after reload.
const nonceWords = new Uint32Array(2);
if (window.crypto?.getRandomValues) window.crypto.getRandomValues(nonceWords);
else {
  // A correlation ID, not an authentication secret; support older kiosk/test
  // contexts without Web Crypto without breaking even the ordinary home UI.
  nonceWords.set([Date.now() >>> 0, Math.floor(Math.random() * 0x100000000)]);
}
const documentNonce = Array.from(nonceWords,
  value => value.toString(16).padStart(8, '0')).join('');

export function navigationGraphicsBridge() {
  return window.webkit?.messageHandlers?.camrodNavigationGraphics;
}

export function acquireNavigationGraphics(onReady, onFailure) {
  const bridge = navigationGraphicsBridge();
  if (!bridge) { onReady(); return () => {}; }
  const id = `nav-${documentNonce}-${++sequence}`;
  let released = false;
  let settled = false;
  const clean = () => {
    clearTimeout(timeout);
    window.removeEventListener('camrod-navigation-graphics', onReply);
  };
  const release = () => {
    if (released) return;
    released = true;
    clean();
    // Release after React has disposed the renderer; token ownership also
    // keeps a StrictMode remount's new lease alive during the old cleanup.
    Promise.resolve().then(() => {
      try { bridge.postMessage(JSON.stringify({ id, active: false })); } catch (_) { /* closing window */ }
    });
  };
  const onReply = event => {
    if (released || settled || event.detail?.id !== id) return;
    settled = true;
    clean();
    if (event.detail.enabled === true) onReady();
    else { release(); onFailure(); }
  };
  const timeout = setTimeout(() => {
    if (!released && !settled) { settled = true; release(); onFailure(); }
  }, 5000);
  window.addEventListener('camrod-navigation-graphics', onReply);
  try { bridge.postMessage(JSON.stringify({ id, active: true })); }
  catch (_) { settled = true; release(); onFailure(); }
  return release;
}
