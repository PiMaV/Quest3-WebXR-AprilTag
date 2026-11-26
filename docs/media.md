# Demo Media Integration Guide

Three demo videos are produced separately for the **Quest3-WebXR-AprilTag** public release. Neither [`README.md`](../README.md) nor [`index.html`](../index.html) links to non-existent video files or placeholder URLs while production is in progress.

---

## Planned Media Files

| Asset | Target Path | Target Length | Intended Role |
|---|---|---|---|
| **Hook clip** | `media/hook.mp4` | ~10 s | Silent looping hero preview (`muted`, `autoplay`, `loop`, `playsinline`) |
| **Launch clip** | `media/launch.mp4` | ~50 s | Concise feature & tracking overview (with audio & player controls) |
| **Full demo** | `media/demo-full.mp4` | ~3 min | Complete Quest 3 walkthrough: calibration, native lock, `SCAN`, `MEASURE` |

---

## How to Activate Once MP4 Files Are Delivered

### 1. HTML Landing Page ([`index.html`](../index.html))

Place the delivered MP4 files under `media/` and update the `DEMO_MEDIA` configuration inside the inline `<script>` in [`index.html`](../index.html):

```js
const DEMO_MEDIA = {
  hook: './media/hook.mp4',
  launch: './media/launch.mp4',
  full: './media/demo-full.mp4',
};
```

As soon as any entry is non-`null`, the video stage in `#demo-video` automatically switches from the static architectural slate to a native HTML5 `<video playsinline preload="metadata">` player with clip switcher controls (`hook` plays muted/looping; `launch` and `full` show native playback controls).

### 2. Local Static Server

Ensure [`scripts/serve.mjs`](../scripts/serve.mjs) serves `.mp4` with `Content-Type: video/mp4` and HTTP `Range` (`206 Partial Content`) support so seeking works cleanly in Desktop, mobile, and Quest Browser during local previews.

### 3. [`README.md`](../README.md)

Locate the `<!-- MEDIA_SLOT -->` comment under `## Demo` in [`README.md`](../README.md) and either:
- Upload the MP4 directly into GitHub's Markdown editor (GitHub hosts native `<video>` embeds automatically), or
- Link the repository asset path / release media URL directly inside the `<!-- MEDIA_SLOT -->` block.

---

## Recommended Encoding for Desktop, Mobile & Quest Browser

For broad compatibility across Desktop browsers, iOS/Android Safari/Chrome, and Meta Quest Browser:

```bash
ffmpeg -i input.mov \
  -c:v libx264 -profile:v high -level 4.1 -pix_fmt yuv420p \
  -crf 21 -movflags +faststart \
  -c:a aac -b:a 128k \
  media/launch.mp4
```

For the silent `media/hook.mp4` (~10 s loop), strip the audio track with `-an` so `autoplay muted loop playsinline` engages instantaneously.
