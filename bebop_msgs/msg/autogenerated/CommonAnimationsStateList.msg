# CommonAnimationsStateList
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/common_commands.xml
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: List of animations state.

Header header

# Animation type.
uint8 anim_HEADLIGHTS_FLASH=0  # Flash headlights.
uint8 anim_HEADLIGHTS_BLINK=1  # Blink headlights.
uint8 anim_HEADLIGHTS_OSCILLATION=2  # Oscillating headlights.
uint8 anim_SPIN=3  # Spin animation.
uint8 anim_TAP=4  # Tap animation.
uint8 anim_SLOW_SHAKE=5  # Slow shake animation.
uint8 anim_METRONOME=6  # Metronome animation.
uint8 anim_ONDULATION=7  # Standing dance animation.
uint8 anim_SPIN_JUMP=8  # Spin jump animation.
uint8 anim_SPIN_TO_POSTURE=9  # Spin that end in standing posture, or in jumper if it was standing animation.
uint8 anim_SPIRAL=10  # Spiral animation.
uint8 anim_SLALOM=11  # Slalom animation.
uint8 anim_BOOST=12  # Boost animation.
uint8 anim
# State of the animation
uint8 state_stopped=0  # animation is stopped
uint8 state_started=1  # animation is started
uint8 state_notAvailable=2  # The animation is not available
uint8 state
# Error to explain the state
uint8 error_ok=0  # No Error
uint8 error_unknown=1  # Unknown generic error
uint8 error
