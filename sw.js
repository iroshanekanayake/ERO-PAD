const CACHE_NAME = "ero-pad-os-shell-v2";
const APP_SHELL = [
  "./",
  "./index.html",
  "./manifest.json",
  "./icon-192.png",
  "./icon-512.png"
];

self.addEventListener("install", (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then((cache) => cache.addAll(APP_SHELL))
      .then(() => self.skipWaiting())
  );
});

self.addEventListener("activate", (event) => {
  event.waitUntil(
    caches.keys()
      .then((keys) => Promise.all(
        keys.filter((key) => key.startsWith("ero-pad-os-shell-") && key !== CACHE_NAME)
          .map((key) => caches.delete(key))
      ))
      .then(() => self.clients.claim())
  );
});

self.addEventListener("fetch", (event) => {
  const request = event.request;
  if (request.method !== "GET") return;

  const url = new URL(request.url);
  if (url.origin !== self.location.origin) return;

  // Never cache ESP32 control/API endpoints or WebSocket-related requests.
  const path = url.pathname.toLowerCase();
  if (/\/(get-|save-|load-|upload-|delete-|restore-|format-|list-|set-|reset-)/.test(path) ||
      path.endsWith("/ws")) {
    return;
  }

  if (request.mode === "navigate") {
    event.respondWith(
      fetch(request)
        .then((response) => {
          if (response && response.ok) {
            const copy = response.clone();
            caches.open(CACHE_NAME).then((cache) => cache.put("./index.html", copy));
          }
          return response;
        })
        .catch(async () => {
          return (await caches.match("./index.html")) ||
                 (await caches.match("./")) ||
                 new Response("ERO PAD OS is not available offline yet. Open it online once first.",
                   { status: 503, headers: { "Content-Type": "text/plain; charset=utf-8" } });
        })
    );
    return;
  }

  const isAppAsset = /\/(manifest\.json|icon-192\.png|icon-512\.png)$/.test(path);
  if (!isAppAsset) return;

  event.respondWith(
    caches.match(request).then((cached) => cached || fetch(request).then((response) => {
      if (response && response.ok) {
        const copy = response.clone();
        caches.open(CACHE_NAME).then((cache) => cache.put(request, copy));
      }
      return response;
    }))
  );
});
