# Build
`git submodule update --init --recursive`
`make`

# Usage

All six knobs are CV-controllable!
### FFT Controls:
- Warp: offset FFT bands (=multiple bins) by a number of bins
- Blur: Change number of bins in each band
- Mix: 0-50%: attenuate the bands between 0-100%.
- Mix: 50-100%: Boost the bands between 100-800% (the waveform is also normalized here not to clip, the result currently is it is a lot more silent in this case)

### Comb filter controls:
- Time: delay time in samples (1-16k)
- Reflect: feedback
- Atmosphere: TiltEQ, clock-wise to hipass, ccw to lowpass

### I/O section 
This part could use some more work, right now its essentially only working Left in --> Left out.
- L out: Wet signal
- R out: Unaffected Dry signal
- L in: Used for everything
- R in: Used for nothing!


The code could be a lot more optimized! New features could hence be added pretty easily. 

## Get it
Prebuilt firmware binary (free): https://tsarpf.gumroad.com/l/spectral-comb-filter-aurora-custom-firmware

Demo https://www.youtube.com/watch?v=jmJCHJux5lY

Buy me a coffee if it's useful https://www.buymeacoffee.com/tsurba
