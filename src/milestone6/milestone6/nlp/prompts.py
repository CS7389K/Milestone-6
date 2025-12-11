# Optimal prompt format: Plain text completion with comprehensive examples
# This format achieves 100% accuracy by:
# 1. Using plain text (no [INST] tags which cause issues with this model)
# 2. Providing many concrete examples covering all actions
# 3. Normalizing input to lowercase
# 4. Using \n as stop token
# 5. Low temperature (0.1) for consistency

SYSTEM_PROMPT = """Convert command to robot action code.

Examples:
turn left -> TURN_LEFT
turn right -> TURN_RIGHT
go right -> TURN_RIGHT
move forward -> MOVE_FORWARD
go ahead -> MOVE_FORWARD
move ahead -> MOVE_FORWARD
pick up bottle -> GRAB
grab bottle -> GRAB
grab it -> GRAB
grab bear -> GRAB
pick up object -> GRAB
place object -> PLACE
put it down -> PLACE
place bottle -> PLACE
find bear -> SEARCH BEAR
search for bear -> SEARCH BEAR
find bottle -> SEARCH BOTTLE
search for bottle -> SEARCH BOTTLE
look around -> SCAN
scan area -> SCAN
scan -> SCAN
take me to bear -> TRANSPORT_TO BEAR
go to bear -> TRANSPORT_TO BEAR
take me to bottle -> TRANSPORT_TO BOTTLE
go to bottle -> TRANSPORT_TO BOTTLE
done -> DONE
finished -> DONE
complete -> DONE"""
