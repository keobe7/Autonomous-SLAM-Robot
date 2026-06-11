#!/usr/bin/env python3

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point


class Frontier:
    def __init__(self, size: int, centroid: Point):
        self.size = size
        self.centroid = centroid


class FrontierSearch:
    MIN_FRONTIER_SIZE = 8

    @staticmethod
    def get_cell_value(mapdata: OccupancyGrid, p: "tuple[int, int]") -> int:
        index = p[1] * mapdata.info.width + p[0]
        return mapdata.data[index]

    @staticmethod
    def is_in_bounds(mapdata: OccupancyGrid, p: "tuple[int, int]") -> bool:
        return 0 <= p[0] < mapdata.info.width and 0 <= p[1] < mapdata.info.height

    @staticmethod
    def neighbors_4(mapdata: OccupancyGrid, p: "tuple[int, int]") -> "list[tuple[int, int]]":
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        return [
            (p[0] + dx, p[1] + dy)
            for dx, dy in directions
            if FrontierSearch.is_in_bounds(mapdata, (p[0] + dx, p[1] + dy))
        ]

    @staticmethod
    def neighbors_8(mapdata: OccupancyGrid, p: "tuple[int, int]") -> "list[tuple[int, int]]":
        directions = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
        return [
            (p[0] + dx, p[1] + dy)
            for dx, dy in directions
            if FrontierSearch.is_in_bounds(mapdata, (p[0] + dx, p[1] + dy))
        ]

    @staticmethod
    def grid_to_world(mapdata: OccupancyGrid, p: "tuple[int, int]") -> Point:
        x = (p[0] + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x
        y = (p[1] + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y
        return Point(x=x, y=y, z=0.0)

    @staticmethod
    def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> "tuple[int, int]":
        x = int((wp.x - mapdata.info.origin.position.x) / mapdata.info.resolution)
        y = int((wp.y - mapdata.info.origin.position.y) / mapdata.info.resolution)
        return (x, y)

    @staticmethod
    def is_frontier_cell(mapdata: OccupancyGrid, cell: "tuple[int, int]", is_frontier: dict) -> bool:
        if FrontierSearch.get_cell_value(mapdata, cell) != -1:
            return False
        if cell in is_frontier:
            return False
        WALKABLE_THRESHOLD = 50
        for neighbor in FrontierSearch.neighbors_4(mapdata, cell):
            val = FrontierSearch.get_cell_value(mapdata, neighbor)
            if 0 <= val < WALKABLE_THRESHOLD:
                return True
        return False

    @staticmethod
    def build_frontier(
        mapdata: OccupancyGrid,
        initial_cell: "tuple[int, int]",
        is_frontier: dict,
    ) -> "Frontier | None":
        queue = [initial_cell]
        size = 0
        cx = 0.0
        cy = 0.0

        while queue:
            current = queue.pop(0)
            if current in is_frontier:
                continue
            is_frontier[current] = True
            size += 1
            cx += current[0]
            cy += current[1]

            for neighbor in FrontierSearch.neighbors_8(mapdata, current):
                if FrontierSearch.is_frontier_cell(mapdata, neighbor, is_frontier):
                    queue.append(neighbor)

        if size < FrontierSearch.MIN_FRONTIER_SIZE:
            return None

        centroid = FrontierSearch.grid_to_world(mapdata, (int(cx / size), int(cy / size)))
        return Frontier(size=size, centroid=centroid)

    @staticmethod
    def search(
        mapdata: OccupancyGrid,
        start: "tuple[int, int]",
    ) -> "list[Frontier]":
        queue = [start]
        visited = {start: True}
        is_frontier: dict = {}
        frontiers: list[Frontier] = []

        while queue:
            current = queue.pop(0)

            for neighbor in FrontierSearch.neighbors_4(mapdata, current):
                val = FrontierSearch.get_cell_value(mapdata, neighbor)

                if val >= 0 and neighbor not in visited:
                    visited[neighbor] = True
                    queue.append(neighbor)

                elif FrontierSearch.is_frontier_cell(mapdata, neighbor, is_frontier):
                    frontier = FrontierSearch.build_frontier(mapdata, neighbor, is_frontier)
                    if frontier is not None:
                        frontiers.append(frontier)

        return frontiers