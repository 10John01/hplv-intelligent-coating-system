**Intelligent Industrial HVLP Coating System**

Created by John Thomas DuCrest Lock in collaboration with James Lee and multiple AI systems over 9 months of iterative development.

What Happened:

Week 3 of adapting a vertical wall printer for coating work, I discovered proprietary restrictions and imeditealy pivoted away.

I Pitched a complete rebuild to management. Built it from scratch using only off-the-shelf components, a $1000 budget with the intent of making it move like a trained professional coater with machine precision.

**What We Built:**
- **James Lee**: Designed and built the mechanical systems - **Her body**
- **John Thomas DuCrest Lock**: Designed the control software, wired and soldered the entire system with AI collaboration - **Her mind**
- **Metal shop & powder coating teams**: The control box, a blue powder coat finish, laser engraving from the Professional fabrication experts at FX Industries - **Her look**

**Components**: Teensy 4.1, Custom Pneumatic solenoid, Air Handling, Adafruit FRAM, SparkFun sensors, Amazon electronics. 

**Result**: A 66" x 36" dual-axis coating system. Teresa (Lead Coater for FX Industries) operates it daily.

## The Code
4000 lines of working industrial control code. 184 iterations.

- Manual atomic FRAM storage with power-loss protection
- Intelligent spray patterns for consistent coating density  
- Position tracking with drift correction
- Load-compensated ramping for 50-pound spray assembly
- Safety-first design with boom arm located at the bottom. If catastropic failure were to occur the 50 lbs boom arm will fall and stop 1' from the floor via physical stops

## Technical Details
- **Control**: Teensy 4.1 microcontroller
- **Storage**: 512KB FRAM with custom SPI implementation
- **Interface**: 3 alphanumeric displays, 4 encoders, LED feedback
- **Axes**: Y-belt drive, Z-rack & pinion with 10:1 gear reduction
- **Workspace**: 66" x 36" coating area
- **Code**: C++, real-time control, 184 development iterations

## Key Innovations
**Manual Atomic FRAM Storage**: Custom bit-level SPI implementation with transaction validation - mark invalid, write data, verify, mark valid. Prevents corrupted state during power loss.

**Intelligent Spray Pattern Calculation**: Maintains consistent passes-per-inch regardless of substrate size using overlap mathematics: `effectiveStepSize = sprayHeight * (1.0 - overlapFraction)`.

**Position Drift Detection**: Multi-counter tracking with checksum validation detects and corrects accumulated positioning errors automatically.

**Load-Compensated Z-Axis Ramping and gear ratio**: Time-based acceleration curves and a gear ratio of 10:1 optimized for heavy boom arm assembly.

## 🎥 Demo Video

We took a $20K proprietary paperweight and created our own proprietary intelligent HPLV system named Job Security (as the rumor around the FX Industries was, "it" was going to take over Teressa's job, so she named it wisely and humorously) - (12s clip):

**Operated by Teresa, Lead Coater at FX Industries.**

[![Watch the demo on YouTube](https://img.youtube.com/vi/klHbX47O5l0/0.jpg)](https://youtu.be/klHbX47O5l0)

## Credits
- **Lady Ada & Adafruit**: FRAM technology and hardware foundation, always a solid choice for Hardware!
- **SparkFun & Amazon**: Components like sheilded wiring, cable ties, etc.  
- **Multiple AI collaborators**: ChatGPT-o4/5/5 Instant, Claude Sonnet 4, ChatGPT-4.1 API
- **Collaborative development**: Human domain expertise + AI technical analysis through 184 iterations

This is production code that runs real equipment. Every commented section and patch represents decisions made under actual constraints... I chose to leave in promising code and comment it out as this would aid in scaliablity.

## Source Code:
The complete Job Security v18.4 control system: [job_security_v18.3.cpp](https://github.com/10John01/hplv-intelligent-coating-system/blob/main/code/job_security_v18.3.cpp)

