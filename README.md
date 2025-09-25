Intelligent Industrial HPLV Coating System

Created by John Thomas DuCrest Lock in collaboration with James Lee and multiple AI systems over 9 months of iterative development.

What Happened
Week 3 of adapting a vertical wall printer for coating work: discovered proprietary restrictions and pivoted away.

Pitched a complete rebuild to management. Build it from scratch using only off-the-shelf components. $1000 budget. Make it move like a trained professional coater with machine precision.

What We Built
- **James Lee**: Designed and built the mechanical systems - Her body
- **John Thomas DuCrest Lock**: Designed the control software with AI collaboration - Her mind
- **Metal shop & powder coating teams**: Professional fabrication - Her look

**Components**: Teensy 4.1, Adafruit FRAM, SparkFun sensors, Amazon electronics, $20 solder gun...

**Result**: 66" x 36" dual-axis coating system. Teresa (Lead Coater for FX Industries) operates it daily.

## The Code
3000+ lines of working industrial control code. 183 iterations. It's messy because it prioritizes function over form.

- Manual atomic FRAM storage with power-loss protection
- Intelligent spray patterns for consistent coating density  
- Position tracking with drift correction
- Load-compensated ramping for 50-pound spray assembly
- Safety-first design with TRUE HOME at bottom

## Technical Details
- **Control**: Teensy 4.1 microcontroller
- **Storage**: 512KB FRAM with custom SPI implementation
- **Interface**: 3 alphanumeric displays, 4 encoders, LED feedback
- **Axes**: Y-belt drive, Z-rack & pinion with 10:1 gear reduction
- **Workspace**: 66" x 36" coating area
- **Code**: C++, real-time control, 183 development iterations

## Key Innovations
**Manual Atomic FRAM Storage**: Custom bit-level SPI implementation with transaction validation - mark invalid, write data, verify, mark valid. Prevents corrupted state during power loss.

**Intelligent Spray Pattern Calculation**: Maintains consistent passes-per-inch regardless of substrate size using overlap mathematics: `effectiveStepSize = sprayHeight * (1.0 - overlapFraction)`.

**Position Drift Detection**: Multi-counter tracking with checksum validation detects and corrects accumulated positioning errors automatically.

**Load-Compensated Z-Axis Ramping**: Time-based acceleration curves optimized for heavy spray assembly. Solved the 50-pound safety problem by relocating TRUE HOME to bottom - Z-axis only moves up.

## 🎥 Demo Video

A $20K proprietary paperweight, reborn into a $1K intelligent HPLV system - (12s clip):

[![Watch the demo on YouTube](https://img.youtube.com/vi/klHbX47O5l0/0.jpg)](https://youtu.be/klHbX47O5l0)

## Credits
- **Lady Ada & Adafruit**: FRAM technology and hardware foundation, always a solid choice for Hardware!
- **SparkFun & Amazon**: Component ecosystem  
- **Multiple AI collaborators**: ChatGPT-o4/5/5 Instant, Claude Sonnet 4, ChatGPT-4.1 API
- **Collaborative development**: Human domain expertise + AI technical analysis through 183 iterations

This is production code that runs real equipment. Every commented section and patch represents decisions made under actual constraints.

## Source Code:
The complete Job Security v18.3 control system: [job_security_v18.3.cpp](https://github.com/10John01/hplv-intelligent-coating-system/blob/main/code/job_security_v18.3.cpp)

