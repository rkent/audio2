# Run alsa mixer polling in a separate thread

import json
import logging
import os
import time
import selectors
import threading

import alsaaudio as aa

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class MixerThread:
    def __init__(self, control, device, publish_guard=None):
        self.mixer = None
        self.sel = selectors.DefaultSelector()
        self.lock = threading.Lock()
        self._volume = None
        self._muted = None
        self.publish_guard = publish_guard
        self.control = control
        self.device = device
        self.read_fd, self.write_fd = os.pipe()
        self.shutdown_event = threading.Event()
        self.mute_warned = False

    @property
    def ready(self) -> bool:
        """Indicates if the mixer is initialized and ready. Thread-safe."""
        with self.lock:
            return self.mixer is not None

    @property
    def volume(self) -> list[int]:
        """Thread-safe getter for current volume (percentage)."""
        with self.lock:
            return self._volume

    @volume.setter
    def volume(self, value):
        """Thread-safe setter for volume, typically used on main thread.

        value can be an int, which sets all channels to that volume,
        or a list of ints for per-channel volume.
        """
        if self.mixer and value is not None:
            message = {"command": "set_volume", "volume": value}
            os.write(self.write_fd, (json.dumps(message) + '\n').encode('utf-8'))
        else:
            logger.warning("Mixer object is not initialized; cannot set volume.")

    @property
    def muted(self) -> bool:
        """Thread-safe getter for mute state."""
        with self.lock:
            return self._muted

    @muted.setter
    def muted(self, value):
        """Thread-safe setter for mute, typically used on main thread.

        value can be a bool, which sets all channels to that mute state,
        or a list of bools for per-channel mute.
        """
        logger.info("Setting muted to %s in setter", value)

        if self.mixer:
            message = {"command": "set_mute", "mute": value}
            os.write(self.write_fd, (json.dumps(message) + '\n').encode('utf-8'))
            logger.info("Property set: muted -> %s", value)
        else:
            logger.warning("Mixer object is not initialized; cannot set mute.")

    def update_mixer(self) -> None:
        """Update the mixer status and store in thread-safe properties."""
        if self.mixer:
            # Now you can query the mixer for updated information
            mixer_obj = self.mixer
            volume = mixer_obj.getvolume(units=aa.VOLUME_UNITS_PERCENTAGE)
            try:
                muted = mixer_obj.getmute()
                if type(muted) in (list, tuple):
                    muted = all(muted)
                else:
                    muted = bool(muted)
            except aa.ALSAAudioError:
                muted = False  # Assume not muted if unsupported
                if not self.mute_warned:
                    logger.warning("Mixer does not support getmute(); assuming unmuted.")
                    self.mute_warned = True
            with self.lock:
                self._volume = volume
                self._muted = muted

            logging.info(
                f"Mixer event detected! New Volume: {self._volume}%, Muted: {self._muted}"
            )
            if self.publish_guard:
                self.publish_guard.trigger()
        else:
            logging.warning("Mixer object is not initialized.")


    def register_mixer(self, control: str, device: str) -> None:
        '''
        # 1. Initialize the ALSA Mixer object
        # Use the appropriate control name for your system (e.g., 'Master', 'PCM')
        '''
        if self.mixer is not None:
            logger.warning("Mixer is already being monitored.")
            return
        # make sure the device is valid
        playback_devices = aa.pcms(aa.PCM_PLAYBACK)
        capture_devices = aa.pcms(aa.PCM_CAPTURE)
        valid_playback = device in playback_devices
        valid_capture = device in capture_devices
        if not (valid_playback or valid_capture):
            logger.warning(f"Invalid device '{device}' for mixer.")
            logger.info(f"Available playback devices: {playback_devices}")
            logger.info(f"Available capture devices: {capture_devices}")
            return
        # make sure the control is valid
        try:
            avail_controls = aa.mixers(device=device)
        except aa.ALSAAudioError as e:
            logger.error(f"Error retrieving available mixers, device '{device}' probably has no controls': {e}")
            # which devices are available with controls?
            # This may display a lot of errors, which I have been unsuccessful in suppressing.
            logger.info("Checking which devices have available mixer controls. This may show a lot of 'ALSA lib' errors, ignore them")
            valid_devices = set()
            for playback_device in playback_devices:
                try:
                    mixers = aa.mixers(device=playback_device)
                    if mixers:
                        valid_devices.add(playback_device)
                except aa.ALSAAudioError:
                    pass
            for capture_device in capture_devices:
                try:
                    mixers = aa.mixers(device=capture_device)
                    if mixers:
                        valid_devices.add(capture_device)
                except aa.ALSAAudioError:
                    pass
            logger.info(f"Devices with available mixer controls: {valid_devices}")
            return
        if control not in avail_controls:
            logger.warning(f"Invalid control '{control}' for device '{device}'.")
            try:
                avail_controls = aa.mixers(device=device)
                logger.info(f"Available mixer controls on device '{device}': {avail_controls}")
            except aa.ALSAAudioError as e2:
                logger.error(f"Error retrieving available mixers, device '{device}': {e2}")
            return
        try:
            with self.lock:
                self.mixer = aa.Mixer(control=control, device=device)
        except aa.ALSAAudioError as e:
            logger.warning(f"Error opening mixer: {e}")
            try:
                avail_controls = aa.mixers(device=device)
                logger.info(f"Available mixer controls on device '{device}': {avail_controls}")
            except aa.ALSAAudioError as e2:
                logger.error(f"Error retrieving available mixers, device '{device}': {e2}")
            return

        logger.info(f"Monitoring '{self.mixer.mixer()}' control on device '{device}'")

        # Get the pollable file descriptor and event mask
        # polldescriptors() returns a list of tuples: [(fd, event_mask), ...]
        descriptors = self.mixer.polldescriptors()
        if not descriptors:
            logger.warning("Could not get poll descriptors for the mixer. Exiting.")
            return

        # In most cases, there will be only one descriptor
        fd, event_mask = descriptors[0]

        # Register the file descriptor with the selector and associate the callback
        # The 'data' field stores the mixer object so the callback can access it.
        mixer_data = {"params": None, "callback": self.handle_mixer_event}
        self.sel.register(fd, selectors.EVENT_READ, data=mixer_data)
        print("Registered mixer fd with selector.")

    def set_alsa_mute(self, mute: bool, channel: int | None = None) -> None:
        if self.mixer:
            logger.info(f"Set alsa mute to {mute} channel {channel}")
            try:
                if type(channel) is int:
                    self.mixer.setmute(bool(mute), channel)
                else:
                    self.mixer.setmute(bool(mute))
            except aa.ALSAAudioError as e:
                if not self.mute_warned:
                    logger.warning("Mixer does not support setmute(); cannot set mute state.")
                    self.mute_warned = True
        else:
            logger.warning("Mixer object is not initialized; cannot set mute.")

    def set_alsa_volume(self, volume: int, channel: int | None = None) -> None:
        if self.mixer:
            logger.info(f"Set alsa volume to {volume} channel {channel}")
            try:
                if type(channel) is int:
                    self.mixer.setvolume(volume, channel=channel)
                else:
                    self.mixer.setvolume(volume)
            except aa.ALSAAudioError as e:
                logger.error("Error setting alsa volume: %s", e)
        else:
            logger.warning("Mixer object is not initialized; cannot set volume.")

    def handle_mixer_event(self, _) -> None:
        """Callback function to handle the event."""
        # Read the event to clear the ALSA buffer and get the new volume/status
        # We don't necessarily need the return values, but the read() call
        # is necessary to process the event within ALSA's C library.
        if self.mixer:
            self.mixer.handleevents()
            self.update_mixer()
        else:
            logging.warning("Mixer object is not initialized in event handler.")

    def handle_message_event(self, _) -> None:
        read_fd = self.read_fd
        """Callback function to handle data when the read end of the pipe is ready."""
        MAX_BUFFER_SIZE = 1024
        data = os.read(read_fd, MAX_BUFFER_SIZE)
        logger.debug("Received data from pipe of length %d", len(data))
        if not data:
            # If read returns empty bytes, the write end may be closed
            if self.shutdown_event.is_set():
                logger.info("Main thread closed the pipe. Unregistering selector.")
                self.sel.unregister(read_fd)
                os.close(read_fd)
            else:
                logger.info("Shutdown event not set, continuing message handler.")
        elif len(data) > MAX_BUFFER_SIZE:
            logger.error("Data length %d exceeds buffer size", len(data))
            # The remaining data will be lost
            while os.read(read_fd, MAX_BUFFER_SIZE):
                pass
            return
        else:
            if not self.mixer:
                logger.warning("Mixer object is not initialized; cannot process message.")
                return
            data = data.decode('utf-8')
            # More than one message could be received; handle them all
            messages = data.splitlines()
            for jmessage in messages:
                logger.info("Received message: %s", jmessage)
                message = json.loads(jmessage)
                if message.get("command") == "set_mute":
                    mute = message.get("mute", False)
                    if type(mute) in (list, tuple):
                        if all(item == mute[0] for item in mute):
                           self.set_alsa_mute(bool(mute[0]))
                        else:
                            for channel in range(len(mute)):
                                self.set_alsa_mute(bool(mute[channel]), channel)
                    else:
                        self.set_alsa_mute(bool(mute))
                elif message.get("command") == "set_volume":
                    volume = message.get("volume", 50)
                    if type(volume) in (list, tuple):
                        if all(item == volume[0] for item in volume):
                            self.set_alsa_volume(int(volume[0]))
                        else:
                            for channel in range(len(volume)):
                                self.set_alsa_volume(int(volume[channel]), channel)
                    else:
                        self.set_alsa_volume(int(volume))
                else:
                    logger.warning("Unknown command received: %s", message.get("command"))
            # After processing, update the mixer status, since we do not seem to get an event.
            self.update_mixer()

    def run(self):
        """A thread worker to monitor or inititiate ALSA mixer events."""

        # Register the read pipe
        read_data = {"params": self.read_fd, "callback": self.handle_message_event}
        self.sel.register(self.read_fd, selectors.EVENT_READ, data=read_data)
        self.register_mixer(self.control, self.device)
        self.update_mixer()

        # Event loop
        logger.info("Waiting for mixer events (Ctrl+C to exit)...")
        while not self.shutdown_event.is_set():
            # Wait for events without a timeout
            ready_list = self.sel.select()
            if not ready_list:
                logger.info("Empty ready list, closing")
                break

            for key, _ in ready_list:
                # Retrieve the stored data (mixer object and callback)
                stored = key.data
                callback = stored["callback"]

                # Call the handler function
                callback(stored["params"])
        self.sel.close()

    def close(self):
        """Clean up resources. Callable from main thread."""
        logger.info("Closing MixerThread.")
        if self.mixer:
            logger.info("Closing mixer.")
            with self.lock:
                self.mixer = None
        self.shutdown_event.set()
        os.close(self.write_fd)

if __name__ == "__main__":
    control="Speaker"
    device="speaker"
    mixer_thread = MixerThread(control, device)
    thread  = threading.Thread(target=mixer_thread.run)
    thread.start()

    print("Main thread started monitoring mixer. Sending commands...")

    muted = False
    count = 0
    try:
        while True:
            logger.info(f"Main thread doing other work {count}")
            time.sleep(2)
            count += 1
            if count % 5 == 0:
                base_volume = 50 + (count % 50)
                mixer_thread.volume = [base_volume, base_volume + 5]
            else:
                muted = not muted
                mixer_thread.muted = muted

    except KeyboardInterrupt:
        print("Main thread interrupted by user.")
    finally:
        mixer_thread.close()  # This will fire the thread selector to exit
        thread.join()
