// video.currentTime can advance while getUserMedia pixels are frozen in phone AR.
export function grayFingerprint(gray,width,height) {
  let hash=2166136261;
  // Hash every pixel on small frames (e.g. tests); sample ~4096 evenly-spaced pixels on full frames (<0.1 ms).
  const len=gray.length,stride=len<=4096?1:Math.max(1,(len/4096)|0);
  for(let i=0;i<len;i+=stride) hash=Math.imul(hash^gray[i],16777619)>>>0;
  return `${width}x${height}/${hash.toString(16)}`;
}
export function frameEvidence(previous,fingerprint,callbackCount) {
  const callbackAdvanced=Boolean(previous&&callbackCount>previous.callbackCount);
  const pixelsChanged=Boolean(previous&&fingerprint!==previous.fingerprint);
  return {fresh:!previous||callbackAdvanced||pixelsChanged,callbackAdvanced,pixelsChanged,fingerprint,callbackCount};
}

// Fast path for WebCodecs VideoFrame Y-plane grayscale extraction.
export function prepareGrayFrame(gray,width,height,previous,callbackCount) {
  if(!Number.isInteger(width)||!Number.isInteger(height)||width<=0||height<=0||!(gray instanceof Uint8Array)||gray.length!==width*height)
    throw new TypeError('Invalid grayscale camera frame');
  const fingerprint=grayFingerprint(gray,width,height);
  return {gray,evidence:frameEvidence(previous,fingerprint,callbackCount)};
}

// Runs in the detector worker; the XR render loop only transfers the frame/buffer.
export function prepareRgbaFrame(rgba,width,height,previous,callbackCount,outGray=null) {
  if(!Number.isInteger(width)||!Number.isInteger(height)||width<=0||height<=0||rgba.length!==width*height*4)
    throw new TypeError('Invalid RGBA camera frame');
  const count=width*height;
  const gray=outGray instanceof Uint8Array&&outGray.length===count?outGray:new Uint8Array(count);
  for(let i=0,j=0;j<count;i+=4,j++) {
    gray[j]=(rgba[i]*77+rgba[i+1]*150+rgba[i+2]*29)>>8;
  }
  const fingerprint=grayFingerprint(gray,width,height);
  return {gray,evidence:frameEvidence(previous,fingerprint,callbackCount)};
}
