from typing import List, Tuple


class QueueManager:
    def __init__(self, number_of_queues: int, max_queue_length) -> None:
        self.queues: List[List[int]] = [[] for i in range(number_of_queues)]
        self.max_length = max_queue_length

    def advance_queue(self, queue_nr: int, id: int) -> None:
        if len(self.queues[queue_nr]) == 0:
            return

        self.queues[queue_nr].remove(id)

    def add_to_queue(self, queue_nr: int, id: int) -> None:
        self.queues[queue_nr].append(id)

    def person_is_moveable(self, id: int) -> Tuple[bool, int]:
        for idx, queue in enumerate(self.queues):
            if len(queue) == 0:
                continue
            if id == queue[0]:
                return True, idx

        return False, -1

    def queue_is_full(self, queue_nr: int) -> bool:
        return self.max_length <= len(self.queues[queue_nr])
