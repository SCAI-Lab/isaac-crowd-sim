import json
import os
import sys
from random import randint, random
from typing import Dict, List, Set, Tuple

import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), "../.."))


from utils.animation_generator.position import Position
from utils.animation_generator.queue_manager import QueueManager


class Generator:
    def __init__(self, output_file_path: str) -> None:
        self.file_name = output_file_path
        self.queue_waypoints: List[List[Position]] = []
        self.queue_exit_enabled = True
        self.queue_exit: List[List[Position]] = []

        self.waypoints: List[Position] = []

        self.number_of_people = 0
        self.number_of_queues = 0
        self.density: float = 0
        self.area: float = 0
        self.max_length_queue = 0
        self.number_of_actions = 2000

        self.people_str = "a"
        self.start_index = 0

    def set_number_of_people(self, N: int) -> None:
        self.number_of_people = N

    def set_number_of_queues(self, number_of_queues: int) -> None:
        self.number_of_queues = number_of_queues
        self.queue_waypoints = [[] for i in range(self.number_of_queues)]

    def set_max_length_of_queue(self, max_length: int) -> None:
        self.max_length_queue = max_length

    def set_queue_waypoints(self, waypoints: List[Position], queue: int) -> None:
        self.queue_waypoints[queue] = waypoints

    def add_waypoint(self, pos: Position) -> None:
        self.waypoints.append(pos)

    def set_queue_exit(self, pos: List[Position]) -> None:
        self.queue_exit.append(pos)

    def spawn_people(self) -> None:
        # self.curr_position = np.zeros(self.number_of_people)
        self.curr_position = np.ones(self.number_of_people) * self.start_index

        with open(self.file_name, "a") as file:
            for i in range(self.number_of_people):
                pos_idx = int(self.curr_position[i])
                approx_pos = self.waypoints[pos_idx].get_close_position()
                file.write(
                    f"Spawn {self.people_str}{i} {approx_pos[0]} {approx_pos[1]} 0 0\n"
                )

    def setup_queue(self) -> None:
        with open(self.file_name, "a") as file:
            for i in range(self.number_of_queues):
                if len(self.queue_waypoints[i]) < 2:
                    raise Exception(f"Queue {i} has not enough waypoints set")

                file.write(f"Queue queue_{i}\n")

                if len(self.queue_waypoints[i]) == 2:
                    start = self.queue_waypoints[i][0]
                    end = self.queue_waypoints[i][-1]
                    step_size = (end.position - start.position) / self.max_length_queue
                    for x in range(self.max_length_queue):
                        curr_pos = start.position + x * step_size
                        file.write(f"Queue_Spot queue_{i} {x} {curr_pos[0]} {curr_pos[1]} 0 0\n")

                else:
                    if self.max_length_queue != len(self.queue_waypoints[i]):
                        raise Exception(
                            f"Number of waypoints of queue {i} ({len(self.queue_waypoints[i])}) do not match the max length of the queue {self.max_length_queue}"
                        )

                    for idx, waypnt in enumerate(self.queue_waypoints[i]):
                        file.write(
                            f"Queue_Spot queue_{i} {idx} {waypnt.position[0]} {waypnt.position[1]} 0 0\n"
                        )

    def generate_path_file(self) -> None:
        self.spawn_people()

        if self.number_of_queues > 0:
            self.setup_queue()
            self.queue_manager = QueueManager(self.number_of_queues, self.max_length_queue)

        area: float = 0
        for i in range(len(self.waypoints) - 1):
            path_length = float(
                np.linalg.norm(self.waypoints[i + 1].position - self.waypoints[i].position)
            )
            area += path_length * (self.waypoints[i + 1].error + self.waypoints[i].error) / 2

        # print(f"The current density of the simulation is {self.number_of_people/area}")

        queue_exit: Dict[int, int] = {}

        for i in range(self.number_of_actions):
            for person in range(self.number_of_people):
                probability = 0.85 if self.curr_position[person] == self.start_index else 0.7

                curr_pos_id = self.curr_position[person]

                if curr_pos_id % 1 != 0:
                    # Current position is in queue exit
                    queue_nr = queue_exit.pop(person)
                    self.curr_position[person] += 0.5
                    self.go_to_position(person, self.queue_exit[queue_nr][1])
                    continue

                curr_pos_id = int(curr_pos_id)

                if random() <= probability:
                    # advance people by one step
                    if curr_pos_id >= len(self.waypoints):
                        curr_pos_id = 0
                    curr_step = self.waypoints[curr_pos_id]
                    if curr_pos_id + 1 >= len(self.waypoints):
                        next_step = self.waypoints[0]
                    else:
                        next_step = self.waypoints[curr_pos_id + 1]

                    if next_step.queue:
                        queue_nr = randint(0, self.number_of_queues - 1)
                        self.queue_manager.add_to_queue(queue_nr, person)
                        if not self.queue_manager.queue_is_full(queue_nr):
                            self.curr_position[person] += 1
                            with open(self.file_name, "a") as file:
                                file.write(f"{self.people_str}{person} Queue queue_{queue_nr}\n")
                        else:
                            self.wait_at_position(person)

                        continue

                    if curr_step.queue:
                        # Make the advance of the queue less probable P(x) = 0.8*0.5
                        if random() < 0.5:
                            continue

                        # currently in a queue. Check if person in front of queue, otherwise skip action
                        moveable, queue_nr = self.queue_manager.person_is_moveable(person)
                        if not moveable:
                            continue
                        else:
                            self.queue_manager.advance_queue(queue_nr, person)
                            if self.queue_exit_enabled:
                                self.curr_position[person] += 0.5
                                queue_exit[person] = queue_nr
                                with open(self.file_name, "a") as file:
                                    next_pos = self.queue_exit[queue_nr][0]
                                    file.write(
                                        f"{self.people_str}{person} Dequeue queue_{queue_nr} {next_pos.position[0]} {next_pos.position[1]} 0 0\n"
                                    )
                                continue

                            else:
                                self.curr_position[
                                    person
                                ] = 0  # Queue is the last step so the next step will be the first waypoint again
                                approx_pos = next_step.get_close_position()
                                with open(self.file_name, "a") as file:
                                    file.write(
                                        f"{self.people_str}{person} Dequeue queue_{queue_nr} {approx_pos[0]} {approx_pos[1]} 0 0\n"
                                    )
                                continue

                    if curr_pos_id == -1:
                        # If person is at the end of the waypoint list reset with a certain probability
                        self.curr_position[person] = 0
                        self.go_to_position(person, self.waypoints[0])
                        continue

                    self.curr_position[person] = (
                        curr_pos_id + 1 if curr_pos_id != len(self.waypoints) - 2 else -1
                    )
                    self.go_to_position(person, next_step)

                self.wait_at_position(person)

    def go_to_position(self, person_id: int, pos: Position) -> None:
        approx_pos = pos.get_close_position()
        with open(self.file_name, "a") as file:
            file.write(f"{self.people_str}{person_id} GoTo {approx_pos[0]} {approx_pos[1]} 0 _\n")

    def wait_at_position(self, person_id: int) -> None:
        with open(self.file_name, "a") as file:
            wait_time = randint(0, 5)
            file.write(f"{self.people_str}{person_id} LookAround {wait_time}\n")


if __name__ == "__main__":
    file_name = "static"
    local_root_path = os.path.dirname(os.path.abspath(__file__))
    output_path = f"{local_root_path}/../../config/scenes/{file_name}.txt"

    with open(f"{local_root_path}/../../config/scenes/{file_name}.json") as file:
        config = json.load(file)

    num_generations = config["num_gen"]

    for gen in range(num_generations):
        data = config[f"gen_{gen}"]

        generator = Generator(output_path)
        generator.set_number_of_people(data["num_people"])
        generator.set_max_length_of_queue(data["max_queue"])
        generator.set_number_of_queues(data["num_queues"])

        generator.people_str = chr(ord("a") + gen)  # Have a unique name for each person generated

        pos_x = data["pos_x"]
        pos_y = data["pos_y"]
        savety_margin = data["savety_margin"]

        generator.queue_exit_enabled = data["queue_exit_enabled"]

        for k in range(len(pos_x)):
            generator.add_waypoint(Position(pos_x[k], pos_y[k], False, savety_margin[k]))

        num_of_queues = data["num_queues"]

        if num_of_queues:
            generator.add_waypoint(Position(0, 0, True))  # Dummy waypoint to visit queue

        for i in range(num_of_queues):
            queue_data = data["queues"][f"queue_{i}"]
            pos_x = queue_data["pos_x"]
            pos_y = queue_data["pos_y"]
            generator.set_queue_waypoints(
                [Position(pos_x[k], pos_y[k], True, 1.0) for k in range(len(pos_x))], i
            )

            if data["queue_exit_enabled"]:
                pos_x = queue_data["queue_exit"]["pos_x"]
                pos_y = queue_data["queue_exit"]["pos_y"]
                generator.set_queue_exit([Position(pos_x[k], pos_y[k]) for k in range(len(pos_x))])

        generator.generate_path_file()
