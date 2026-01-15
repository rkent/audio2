# Command to play a file
string path
string topic

# Whether to repeat a file
int8 PLAY_STOP = 0 # Stop this sound from playing
int8 PLAY_ONCE = 1 # Play the sound once
int8 PLAY_START = 2 # Play the sound in a loop until a stop request occurs
int8 play_type
