--[[
This file is part of Courseplay (https://github.com/Courseplay/courseplay)
Copyright (C) 2019 Peter Vaiko

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
]]

PathfinderUtil = {}

--- Calculate the four corners of a rectangle around a node (for example the area covered by a vehicle)
function PathfinderUtil.getCorners(node, length, width, name)
    local x, y, z
    local vehicleData = { node = node, width = width, length = length, name = name,
                          corners = {}}
    x, y, z = localToWorld(node, - width / 2, 0, - length / 2)
    table.insert(vehicleData.corners, {x = x, y = y, z = z})
    x, y, z = localToWorld(node, - width / 2, 0, length / 2)
    table.insert(vehicleData.corners, {x = x, y = y, z = z})
    x, y, z = localToWorld(node, width / 2, 0, length / 2)
    table.insert(vehicleData.corners, {x = x, y = y, z = z})
    x, y, z = localToWorld(node, width / 2, 0, - length / 2)
    table.insert(vehicleData.corners, {x = x, y = y, z = z})
    return vehicleData
end

function PathfinderUtil.findCollidingVehicles(myNode, length, width)
    if not PathfinderUtil.vehicleCollisionData then return false end
    local myVehicle = PathfinderUtil.getCorners(myNode, length, width, 'me')
    for _, collisionData in pairs(PathfinderUtil.vehicleCollisionData) do
        if PathfinderUtil.doRectanglesOverlap(myVehicle.corners, collisionData.corners) then
            return true, collisionData.name
        end
    end
    return false
end

--- Find all other vehicles and add them to our list of vehicles to avoid. Must be called before each pathfinding to 
--- have the current position of the vehicles.
function PathfinderUtil.setUpVehicleCollisionData(myVehicle)
    PathfinderUtil.vehicleCollisionData = {}
    local myRootVehicle = myVehicle and myVehicle:getRootVehicle() or nil
    for _, vehicle in pairs(g_currentMission.vehicles) do
        if vehicle:getRootVehicle() ~= myRootVehicle and vehicle.rootNode and vehicle.sizeWidth and vehicle.sizeLength then
            table.insert(PathfinderUtil.vehicleCollisionData, PathfinderUtil.getCorners(vehicle.rootNode, vehicle.sizeLength, vehicle.sizeWidth, vehicle:getName()))
        end
    end
end

--- This is a simplified implementation of the Separating Axis Test, based on Stephan Schloesser's code in AutoDrive.
--- The implementation assumes that a and b are rectangles (not any polygon)
--- We use this during the pathfinding to drive around other vehicles
function PathfinderUtil.doRectanglesOverlap(a, b)

    if math.abs(a[1].x - b[1].x )> 50 then return false end

    for _, rectangle in pairs({a, b}) do

        -- leverage the fact that rectangles have parallel edges, only need to check the first two
        for i = 1, 2 do
            --grab 2 vertices to create an edge
            local p1 = rectangle[i]
            local p2 = rectangle[i + 1]

            -- find the line perpendicular to this edge
            local normal = {x = p2.z - p1.z, z = p1.x - p2.x}

            local minA = math.huge
            local maxA = -math.huge

            -- for each vertex in the first shape, project it onto the line perpendicular to the edge
            -- and keep track of the min and max of these values
            for _, corner in pairs(a) do
                local projected = normal.x * corner.x + normal.z * corner.z
                if projected < minA then
                    minA = projected
                end
                if projected > maxA then
                    maxA = projected
                end
            end

            --for each vertex in the second shape, project it onto the line perpendicular to the edge
            --and keep track of the min and max of these values
            local minB = math.huge
            local maxB = -math.huge
            for _, corner in pairs(b) do
                local projected = normal.x * corner.x + normal.z * corner.z
                if projected < minB then
                    minB = projected
                end
                if projected > maxB then
                    maxB = projected
                end
            end
            -- if there is no overlap between the projections, the edge we are looking at separates the two
            -- rectangles, and we know there is no overlap
            if maxA < minB or maxB < minA then
                return false
            end
        end
    end
    return true
end

--- Calculate penalty for this node. The penalty will be added to the cost of the node. This allows for
--- obstacle avoidance or forcing the search to remain in certain areas.
---@param node State3D
function PathfinderUtil.getNodePenalty(node)
    -- tweak these two parameters to set up how far the path will be from the field or fruit boundary
    -- size of the area to check for field/fruit
    local areaSize = 3
    -- minimum ratio of the area checked must be on field/clear of fruit
    local minRequiredAreaRatio = 0.8
    local penalty = 0
    local isField, area, totalArea = courseplay:isField(node.x, -node.y, areaSize, areaSize)
    if area / totalArea < minRequiredAreaRatio then
        penalty = penalty + 1000
    end
    if isField then
        local hasFruit
        hasFruit, _, area, totalArea = courseplay:areaHasFruit(node.x, -node.y, nil, areaSize, areaSize)
        if hasFruit and area / totalArea > 1 - minRequiredAreaRatio then
            penalty = penalty + 50
        end
    end
    return penalty
end

--- Check if node is valid: would we collide with another vehicle here?
---@param node State3D
function PathfinderUtil.isValidNode(node, length, width)
    if not PathfinderUtil.helperNode then
        PathfinderUtil.helperNode = courseplay.createNode('pathfinderHelper', node.x, -node.y, 0)
    end
    local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, node.x, 0, -node.y);
    setTranslation(PathfinderUtil.helperNode, node.x, y, -node.y)
    setRotation(PathfinderUtil.helperNode, 0, courseGenerator.toCpAngle(node.t), 0)
    return not PathfinderUtil.findCollidingVehicles(PathfinderUtil.helperNode, length, width)
end

--- Interface function to start the pathfinder
---@param start State3D start node
---@param goal State3D goal node
---@param length number length of the vehicle
---@param width number width of the vehicle, used with length to find nodes invalid due to collision
---@param turnRadius number turn radius of the vehicle
---@param allowReverse boolean allow reverse driving
function PathfinderUtil.startPathfinding(start, goal, length, width, turnRadius, allowReverse)
    PathfinderUtil.setUpVehicleCollisionData()
    local pathfinder = HybridAStarWithAStarInTheMiddle(20)
    local done, path = pathfinder:start(start, goal, length, width, turnRadius, allowReverse, PathfinderUtil.getNodePenalty, PathfinderUtil.isValidNode)
    return pathfinder, done, path
end

function PathfinderUtil:getNodePositionAndDirection(node, sideOffset)
    local x, _, z = localToWorld(node, sideOffset or 0, 0, 0)
    local lx, _, lz = localDirectionToWorld(node, 0, 0, 1)
    local yRot = math.atan2(lx, lz)
    return x, z, yRot
end

--- Interface function to start the pathfinder in the game
---@param vehicle  vehicle, will be used as the start location/heading, turn radius and size
---@param goalWaypoint Waypoint The destination waypoint (x, z, angle)
---@param allowReverse boolean allow reverse driving
function PathfinderUtil.startPathfindingFromVehicleToWaypoint(vehicle, goalWaypoint, allowReverse)
    local x, z, yRot = self:getNodePositionAndDirection(AIDriverUtil.getDirectionNode(vehicle))
    local start = State3D(x, -z, courseGenerator.fromCpAngle(yRot))
    local goal = State3D(goalWaypoint.x, -goalWaypoint.z, courseGenerator.fromCpAngle(goalWaypoint.angle))
    return PathfinderUtil.startPathfinding(start, goal, vehicle.sizeLength, vehicle.sizeWidth, vehicle.cp.turnDiameter / 2, allowReverse)
end

--- Interface function to start the pathfinder in the game. The goal is a point at sideOffset meters from the goal node
--- (sideOffset > 0 is left)
---@param vehicle  vehicle, will be used as the start location/heading, turn radius and size
---@param goalNode node The goal node
---@param sideOffset number side offset of the goal from the goal node
---@param allowReverse boolean allow reverse driving
function PathfinderUtil.startPathfindingFromVehicleToNode(vehicle, goalNode, sideOffset, allowReverse)
    local x, z, yRot = self:getNodePositionAndDirection(AIDriverUtil.getDirectionNode(vehicle))
    local start = State3D(x, -z, courseGenerator.fromCpAngle(yRot))
    x, z, yRot = self:getNodePositionAndDirection(goalNode, sideOffset)
    local goal = State3D(x, -z, courseGenerator.fromCpAngle(yRot))
    return PathfinderUtil.startPathfinding(start, goal, vehicle.sizeLength, vehicle.sizeWidth, vehicle.cp.turnDiameter / 2, allowReverse)
end