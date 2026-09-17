// Run: node tests/test_camera_stream.cjs
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const file = process.argv[2] || path.join(__dirname, '../calibration_replay/static/app.js');
const source = fs.readFileSync(file, 'utf8');
const start = source.indexOf('    const cam = reactive(');
const end = source.indexOf('    // 计划的 base_url', start);
assert(start >= 0 && end > start);
const revoked = [], blobs = [], sockets = [];
const context = {
  reactive: x => x, Blob, ArrayBuffer, JSON,
  setTimeout: () => 1, clearTimeout: () => {},
  URL: {
    createObjectURL(blob) { blobs.push(blob); return `blob:test-${blobs.length}`; },
    revokeObjectURL(url) { revoked.push(url); },
  },
  WebSocket: class {
    constructor(url) { this.url = url; sockets.push(this); }
    close() { this.closed = true; }
    send() {}
  },
};
vm.createContext(context);
vm.runInContext(source.slice(start, end) + '\ncamUrl = "ws://test/ws/stream"; camConnect(); globalThis.test = {cam, camClose, camConnect};', context);
const socket = sockets[0], cam = context.test.cam;
socket.onmessage({data: new Blob([new Uint8Array([255, 216, 255])])});
assert.equal(cam.src, 'blob:test-1');
assert.equal(cam.detected, null);
assert.equal(blobs[0].type, 'image/jpeg');
assert.equal(blobs[0].size, 3);
socket.onmessage({data: new Uint8Array([255, 216, 255]).buffer});
assert.equal(cam.src, 'blob:test-2');
assert.deepEqual(revoked, ['blob:test-1']);
socket.onmessage({data: JSON.stringify({type: 'checkerboard_detection', found: true, corner_count: 88})});
assert.equal(cam.detected, true);
socket.onmessage({data: JSON.stringify({left: '/9j/', left_detected: true})});
assert.equal(cam.src, 'data:image/jpeg;base64,/9j/');
assert.equal(cam.detected, true);
assert.deepEqual(revoked, ['blob:test-1', 'blob:test-2']);
socket.onmessage({data: 'invalid-json'});
assert.equal(cam.src, 'data:image/jpeg;base64,/9j/');
socket.onmessage({data: new Blob(['frame'])});
assert.equal(cam.detected, true);
context.test.camClose();
assert.equal(cam.src, '');
assert.equal(socket.closed, true);
assert.equal(socket.onmessage, null);
assert.equal(socket.onopen, null);
assert.deepEqual(revoked, ['blob:test-1', 'blob:test-2', 'blob:test-3']);
context.test.camConnect();
assert.equal(sockets.length, 2);
sockets[1].onmessage({data: new Blob(['new camera'])});
assert.equal(cam.src, 'blob:test-4');
context.test.camClose();
assert.equal(revoked.length, 4);
console.log('PASS: JPEG frames, checkerboard status, legacy JSON, URL cleanup, reconnect');
