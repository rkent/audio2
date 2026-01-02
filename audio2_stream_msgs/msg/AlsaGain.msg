# Represents the gain of a particular Alsa audio control
string control    # the name given to the volume control
string device     # the alsa device name that contains the control
int32[] percent   # gain expressed as percent of full scale [0 - 100] per channel
bool muted        # is the control muted?
