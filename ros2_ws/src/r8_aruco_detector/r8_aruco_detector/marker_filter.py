class MarkerFilter:
    def __init__(self, required_frames=3):
        self.required_frames = required_frames
        self.counters = {}
        self.previous_ids = set()

    def update(self, detected_ids):
        current_ids = set(detected_ids)

        for marker_id in current_ids:
            if marker_id in self.previous_ids:
                self.counters[marker_id] += 1
            else:
                self.counters[marker_id] = 1

        for marker_id in self.previous_ids - current_ids:
            self.counters[marker_id] = 0

        self.previous_ids = current_ids

        return sorted(
            marker_id
            for marker_id in current_ids
            if self.counters[marker_id] >= self.required_frames
        )
