import { acquireNavigationGraphics } from './navigationGraphicsLease';

// HH_261006 - Exercise native opt-in, failure and cleanup without a GPU/robot.
afterEach(() => { delete window.webkit; jest.useRealTimers(); });
const native = () => {
  const postMessage = jest.fn();
  window.webkit = { messageHandlers: { camrodNavigationGraphics: { postMessage } } };
  return postMessage;
};
const reply = (id, enabled) => window.dispatchEvent(new CustomEvent(
  'camrod-navigation-graphics', { detail: { id, enabled } },
));

test('ordinary browsers retain their existing renderer path', () => {
  const ready = jest.fn();
  const release = acquireNavigationGraphics(ready, jest.fn());
  expect(ready).toHaveBeenCalledTimes(1);
  release();
});

test('native graphics waits for matching ack and is released once', async () => {
  const post = native(), ready = jest.fn(), failure = jest.fn();
  const release = acquireNavigationGraphics(ready, failure);
  const request = JSON.parse(post.mock.calls[0][0]);
  expect(request.active).toBe(true);
  expect(ready).not.toHaveBeenCalled();
  reply('nav-99999', true);
  expect(ready).not.toHaveBeenCalled();
  reply(request.id, true);
  expect(ready).toHaveBeenCalledTimes(1);
  release(); release();
  await Promise.resolve();
  expect(JSON.parse(post.mock.calls[1][0])).toEqual({ id: request.id, active: false });
  expect(post).toHaveBeenCalledTimes(2);
  expect(failure).not.toHaveBeenCalled();
});

test('timed out native request fails explicitly and releases permission', async () => {
  jest.useFakeTimers();
  const post = native(), ready = jest.fn(), failure = jest.fn();
  acquireNavigationGraphics(ready, failure);
  jest.advanceTimersByTime(5000);
  await Promise.resolve();
  expect(ready).not.toHaveBeenCalled();
  expect(failure).toHaveBeenCalledTimes(1);
  expect(JSON.parse(post.mock.calls[1][0]).active).toBe(false);
});

test('unmount before ack does not initialize an orphan renderer', async () => {
  const post = native(), ready = jest.fn();
  const release = acquireNavigationGraphics(ready, jest.fn());
  const { id } = JSON.parse(post.mock.calls[0][0]);
  release(); reply(id, true);
  await Promise.resolve();
  expect(ready).not.toHaveBeenCalled();
});

test('denied native permission never creates a renderer', async () => {
  const post = native(), ready = jest.fn(), failure = jest.fn();
  acquireNavigationGraphics(ready, failure);
  reply(JSON.parse(post.mock.calls[0][0]).id, false);
  await Promise.resolve();
  expect(ready).not.toHaveBeenCalled();
  expect(failure).toHaveBeenCalledTimes(1);
  expect(JSON.parse(post.mock.calls[1][0]).active).toBe(false);
});

test('bridge exception fails explicitly without leaking listeners', async () => {
  const post = native(), failure = jest.fn();
  post.mockImplementation(() => { throw new Error('closed'); });
  const release = acquireNavigationGraphics(jest.fn(), failure);
  release();
  await Promise.resolve();
  expect(failure).toHaveBeenCalledTimes(1);
});
