/* Video element -> pixel helpers shared by the phone and the workstation.
 *
 * The preview is letterboxed: the video is scaled to fit its CSS box and
 * centred, so a client coordinate is not the same as a camera pixel.
 */
(function () {
  "use strict";

  function placement(video) {
    const rect = video.getBoundingClientRect();
    const width = video.videoWidth;
    const height = video.videoHeight;
    if (!width || !height) return null;
    const scale = Math.min(rect.width / width, rect.height / height);
    return {
      rect: rect,
      width: width,
      height: height,
      scale: scale,
      offsetX: rect.left + (rect.width - width * scale) / 2,
      offsetY: rect.top + (rect.height - height * scale) / 2,
    };
  }

  function videoPoint(video, clientX, clientY) {
    const frame = placement(video);
    if (!frame) return null;
    const u = (clientX - frame.offsetX) / frame.scale;
    const v = (clientY - frame.offsetY) / frame.scale;
    if (u < 0 || v < 0 || u > frame.width || v > frame.height) return null;
    return { u: Math.round(u * 100) / 100, v: Math.round(v * 100) / 100 };
  }

  function viewportPoint(video, u, v) {
    const frame = placement(video);
    if (!frame) return null;
    return {
      x: frame.offsetX + u * frame.scale,
      y: frame.offsetY + v * frame.scale,
    };
  }

  function screenPoint(video, stage, u, v) {
    const frame = placement(video);
    if (!frame) return null;
    const host = stage.getBoundingClientRect();
    return {
      x: frame.offsetX - host.left + u * frame.scale,
      y: frame.offsetY - host.top + v * frame.scale,
    };
  }

  function drawFrame(video) {
    const width = video.videoWidth;
    const height = video.videoHeight;
    if (!width || !height) throw new Error("the camera has no frame yet");
    const canvas = document.createElement("canvas");
    canvas.width = width;
    canvas.height = height;
    canvas.getContext("2d").drawImage(video, 0, 0, width, height);
    return canvas;
  }

  function grabImageData(video) {
    const canvas = drawFrame(video);
    return {
      data: canvas.getContext("2d").getImageData(0, 0, canvas.width, canvas.height).data,
      width: canvas.width,
      height: canvas.height,
    };
  }

  async function grabBlob(video, type, quality) {
    const canvas = drawFrame(video);
    const blob = await new Promise((resolve, reject) => {
      canvas.toBlob((value) => (value ? resolve(value) : reject(new Error("JPEG encode failed"))),
        type || "image/jpeg", quality === undefined ? 0.95 : quality);
    });
    return { blob: blob, width: canvas.width, height: canvas.height };
  }

  if (typeof window !== "undefined") {
    window.RebotVideo = {
      placement: placement,
      videoPoint: videoPoint,
      viewportPoint: viewportPoint,
      screenPoint: screenPoint,
      drawFrame: drawFrame,
      grabImageData: grabImageData,
      grabBlob: grabBlob,
    };
  }
  if (typeof module !== "undefined" && module.exports) {
    module.exports = {
      placement: placement,
      videoPoint: videoPoint,
      viewportPoint: viewportPoint,
      screenPoint: screenPoint,
      drawFrame: drawFrame,
      grabImageData: grabImageData,
      grabBlob: grabBlob,
    };
  }
})();
