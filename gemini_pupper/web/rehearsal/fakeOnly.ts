/** Rehearsal-only: never use this fetch adapter in the production application. */
export function loopbackBase(value: string): string {
  const url = new URL(value);
  if (!['http:', 'https:'].includes(url.protocol) || !['localhost', '127.0.0.1', '[::1]'].includes(url.hostname)
    || url.username || url.password || url.search || url.hash || url.pathname !== '/') {
    throw new Error('Rehearsal requires a loopback HTTP origin without credentials, path, query or fragment.');
  }
  return url.origin;
}
export async function assertFake(base: string, rawFetch: typeof fetch): Promise<void> {
  const reply = await rawFetch(`${base}/health`, { signal: AbortSignal.timeout(3000) });
  const health = await reply.json();
  if (!reply.ok || health.runtime !== 'fake' || health.hardware_connected !== false || health.tools !== true
    || health.command_epoch_required !== true || typeof health.command_epoch !== 'string' || !health.command_epoch) {
    throw new Error('Refused: target is not an initialized fake API with the command-epoch protocol.');
  }
}
export function fakeOnlyFetch(base: string, rawFetch: typeof fetch): typeof fetch {
  return async (input, init) => {
    const url = new URL(typeof input === 'string' ? input : input instanceof URL ? input.href : input.url);
    if (url.origin !== base) throw new Error('Rehearsal prohibits non-fake-origin network requests.');
    const method = (init?.method ?? (input instanceof Request ? input.method : 'GET')).toUpperCase();
    if (!['GET', 'HEAD'].includes(method)) await assertFake(base, rawFetch);
    return rawFetch(input, init);
  };
}
