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

This implentation of the hybrid A* is based on Karl Kurzer's code and
master thesis. See:

https://github.com/karlkurzer/path_planner

@mastersthesis{kurzer2016,
  author       = {Karl Kurzer},
  title        = {Path Planning in Unstructured Environments : A Real-time Hybrid A* Implementation for Fast and Deterministic Path Generation for the KTH Research Concept Vehicle},
  school       = {KTH Royal Institute of Technology},
  year         = 2016,
  month        = 12,
}

]]

---@class HybridAStar
HybridAStar = CpObject(Pathfinder)

--- Motion primitives for node expansions, contains the dx/dy/dt values for
--- driving straight/right/left. The idea is to calculate these once as they are
--- only dependent on the turn radius, and then use the precalculated values during the search.
---@class HybridAstar.MotionPrimitives
HybridAStar.MotionPrimitives = CpObject()
-- forward straight/right/left
HybridAStar.MotionPrimitiveTypes = {FS = 'FS', FR = 'FR', FL = 'FL', RS = 'RS', RR = 'RR', RL = 'RL', LL = 'LL', RR = 'RR'}

---@param r number turning radius
---@param expansionDegree number degrees of arc in one expansion step
---@param allowReverse boolean allow for reversing
function HybridAStar.MotionPrimitives:init(r, expansionDegree, allowReverse)
	-- distance travelled in one expansion step (length of an expansionDegree arc of a circle with radius r)
	local d = 2 * r * math.pi * expansionDegree / 360
	-- heading (theta) change in one step
	local dt = math.rad(expansionDegree)
	local dx = r * math.sin(dt)
	local dy = r - r * math.cos(dt)
	-- motion primitive table:
	self.primitives = {}
	-- forward right
	table.insert(self.primitives, {dx = dx, dy = -dy, dt = dt, d = d, type = HybridAStar.MotionPrimitiveTypes.FR})
	-- forward left
	table.insert(self.primitives, {dx = dx, dy = dy, dt = -dt, d = d, type = HybridAStar.MotionPrimitiveTypes.FL})
	-- forward straight
	table.insert(self.primitives, {dx = d, dy = 0, dt = 0, d = d, type = HybridAStar.MotionPrimitiveTypes.FS})
	if allowReverse then
		-- reverse straight
		table.insert(self.primitives, {dx = -d, dy = 0, dt = 0, d = d, type = HybridAStar.MotionPrimitiveTypes.RS})
		-- reverse right
		table.insert(self.primitives, {dx = -dx, dy = -dy, dt = dt, d = d, type = HybridAStar.MotionPrimitiveTypes.RR})
		-- reverse left
		table.insert(self.primitives, {dx = -dx, dy = dy, dt = -dt, d = d, type = HybridAStar.MotionPrimitiveTypes.RL})
	end
end

function HybridAStar.MotionPrimitives.isSameDirection(p1, p2)
	return p1.type:byte(1) == p2.type:byte(1)
end

function HybridAStar.MotionPrimitives.isReverse(p1)
	return p1.type:byte(1) == string.byte('R')
end

function HybridAStar.MotionPrimitives.isTurn(p1)
	return p1.type:byte(2) ~= string.byte('S')
end

function HybridAStar.MotionPrimitives:__tostring()
	local output = ''
	for i, primitive in ipairs(self.primitives) do
		output = output .. string.format('%d: dx: %.4f dy: %.4f dt: %.4f\n', i, primitive.dx, primitive.dy, primitive.dt)
	end
	return output
end

function HybridAStar.MotionPrimitives:getPrimitives()
	return self.primitives
end

--- Motion primitives for a simple A Star algorithm
HybridAStar.SimpleMotionPrimitives = CpObject(HybridAStar.MotionPrimitives)

---@param gridSize number search grid size in meters
function HybridAStar.SimpleMotionPrimitives:init(gridSize, allowReverse)
	local dx = gridSize
	local dy = gridSize
	local d = math.sqrt(dx * dx + dy * dy)
	-- motion primitive table:
	self.primitives = {}
	-- forward right
	table.insert(self.primitives, {dx = dx, dy = -dy, dt = 0, d = d,
								   type = HybridAStar.MotionPrimitiveTypes.FR})
	-- forward left
	table.insert(self.primitives, {dx = dx, dy = dy, dt = 0, d = d,
								   type = HybridAStar.MotionPrimitiveTypes.FL})
	-- forward straight
	table.insert(self.primitives, {dx = dx, dy = 0, dt = 0, d = dx,
								   type = HybridAStar.MotionPrimitiveTypes.FS})
	-- right
	table.insert(self.primitives, {dx = 0, dy = -dy, dt = 0, d = dy,
								   type = HybridAStar.MotionPrimitiveTypes.RR})
	-- left
	table.insert(self.primitives, {dx = 0, dy = dy, dt = 0, d = dy,
								   type = HybridAStar.MotionPrimitiveTypes.LL})
	if allowReverse then
		-- reverse straight
		table.insert(self.primitives, {dx = -dx, dy = 0, dt = 0, d = dx,
									   type = HybridAStar.MotionPrimitiveTypes.RS})
		-- reverse right
		table.insert(self.primitives, {dx = -dx, dy = -dy, dt = 0, d = d,
									   type = HybridAStar.MotionPrimitiveTypes.RR})
		-- reverse left
		table.insert(self.primitives, {dx = -dx, dy = dy, dt = 0, d = d,
									   type = HybridAStar.MotionPrimitiveTypes.RL})
	end
end


---@class HybridAStar.NodeList
HybridAStar.NodeList = CpObject()

--- Configuration space: discretized three dimensional space with x, y and theta coordinates
--- A node with x, y, theta will be assigned to a three dimensional cell in the space
---@param gridSize number size of the cell in the x/y dimensions
---@param thetaResolutionDeg number size of the cell in the theta dimension in degrees
function HybridAStar.NodeList:init(gridSize, thetaResolutionDeg)
	self.nodes = {}
	self.gridSize = gridSize
	self.thetaResolutionDeg = thetaResolutionDeg
	self.lowestCost = math.huge
	self.highestCost = -math.huge
end

---@param node State3D
function HybridAStar.NodeList:getNodeIndexes(node)
	local x = math.floor(node.x / self.gridSize)
	local y = math.floor(node.y / self.gridSize)
	local t = math.floor(math.deg(node.t) / self.thetaResolutionDeg)
	return x, y, t
end

function HybridAStar.NodeList:inSameCell(n1, n2)
	local x1, y1, t1 = self:getNodeIndexes(n1)
	local x2, y2, t2 = self:getNodeIndexes(n2)
	return x1 == x2 and y1 == y2 and t1 == t2
end

---@param node State3D
function HybridAStar.NodeList:get(node)
	local x, y, t = self:getNodeIndexes(node)
	if self.nodes[x] and self.nodes[x][y] then
		return self.nodes[x][y][t]
	end
end

--- Add a node to the configuration space
---@param node State3D
function HybridAStar.NodeList:add(node)
	local x, y, t = self:getNodeIndexes(node)
	if not self.nodes[x] then
		self.nodes[x] = {}
	end
	if not self.nodes[x][y] then
		self.nodes[x][y] = {}
	end
	self.nodes[x][y][t] = node
	if node.cost >= self.highestCost then
		self.highestCost = node.cost
	end
	if node.cost < self.lowestCost then
		self.lowestCost = node.cost
	end
end

function HybridAStar.NodeList:print()
	for _, row in pairs(self.nodes) do
		for _, column in pairs(row) do
			for _, cell in pairs(column) do
				print(cell)
			end
		end
	end
end

function HybridAStar:init()
	self.count = 0
	self.yields = 0
	self.path = {}
	self.iterations = 0
	-- if the goal is within self.deltaPos meters we consider it reached
	self.deltaPos = 1
	-- if the goal heading is within self.deltaThetaDeg degrees we consider it reached
	self.deltaThetaDeg = 5
	-- the same two parameters are used to discretize the continuous state space
end

function HybridAStar:debug(...)
	courseGenerator.debug(...)
end

--- Calculate penalty for this node. The penalty will be added to the cost of the node. This allows for
--- obstacle avoidance or forcing the search to remain in certain areas.
---@param node State3D
function HybridAStar.getNodePenalty(node)
	return 0
end

function HybridAStar:run(...)
	return self:findPath(...)
end

---@param start State3D start node
---@param goal State3D goal node
---@param turnRadius number turn radius of the vehicle
---@param allowReverse boolean allow reverse driving
---@param getNodePenaltyFunc function get penalty for a node, see getNodePenalty()
function HybridAStar:findPath(start, goal, turnRadius, allowReverse, getNodePenaltyFunc)
	if not getNodePenaltyFunc then getNodePenaltyFunc = self.getNodePenalty end
	-- a motion primitive is straight or 10 degree turn to the right or left
	local hybridMotionPrimitives = HybridAStar.MotionPrimitives(turnRadius, 6.75, allowReverse)

	-- create the open list for the nodes as a binary heap where
	-- the node with the lowest total cost is at the top
	local openList = BinaryHeap.minUnique(function(a, b) return a:lt(b) end)

	-- create the configuration space
	---@type HybridAStar.NodeList closedList
	self.nodes = HybridAStar.NodeList(self.deltaPos, self.deltaThetaDeg)

	start:updateH(goal)
	start:insert(openList)
	self.nodes:add(start)

	self.iterations = 0
	self.expansions = 0
	self.yields = 0

	while openList:size() > 0 and self.iterations < 200000 do
		-- pop lowest cost node from queue
		---@type State3D
		local pred = State3D.pop(openList)
		if pred:equals(goal, self.deltaPos, math.rad(self.deltaThetaDeg)) then
			-- done!
			self:rollUpPath(pred, goal)
			return true, self.path
		end

		self.count = self.count + 1
		if self.finder and self.count % 500 == 0 then
			self.yields = self.yields + 1
			coroutine.yield(false)
		end

		if not pred:isClosed() then
			-- create the successor nodes
			local motionPrimitives
			for _, primitive in ipairs(hybridMotionPrimitives:getPrimitives()) do
				---@type State3D
				local succ = pred:createSuccessor(primitive)
				succ:setNodePenalty(getNodePenaltyFunc(succ))
				succ:updateG(primitive)
				succ:updateH(goal, turnRadius)
				local existingSuccNode = self.nodes:get(succ)
				if existingSuccNode then
					-- there is already a node at this (discretized) position
					if not existingSuccNode:isClosed() then
						if existingSuccNode:equals(goal, self.deltaPos, math.rad(self.deltaThetaDeg)) then
							existingSuccNode.pred = succ.pred
							self:rollUpPath(existingSuccNode, goal)
							return true, self.path
						end
						if existingSuccNode:getCost() + 0.1 > succ:getCost() then
							-- successor cell already exist but the new one is cheaper, replace
							-- may add 0.1 to the existing one's cost to prefer replacement
							if self.nodes:inSameCell(succ, pred) then
								existingSuccNode.pred = pred.pred
							end
							if openList:valueByPayload(existingSuccNode) then
								-- existing node is on open list already, replace by removing
								-- it here first
								existingSuccNode:remove(openList)
							end
							-- add (update) to the state space
							self.nodes:add(succ)
							-- add to open list
							succ:insert(openList)
						end
					end
				else
					-- successor cell does not yet exist
					self.nodes:add(succ)
					-- put it on the open list as well
					succ:insert(openList)
				end
			end
			-- node as been expanded, close it to prevent expansion again
			pred:close()
			self.expansions = self.expansions + 1
		end
		self.iterations = self.iterations + 1
	end
	self.path = {}
	self:debug('No path found: iterations %d, expansions %d', self.iterations, self.expansions)
	return true, nil
end

---@param node State3D
function HybridAStar:rollUpPath(node, goal)
	self.path = {}
	local currentNode = node
	self:debug('Goal node at %.2f/%.2f, cost %.1f (%.1f - %.1f)', goal.x, goal.y, node.cost,
			self.nodes.lowestCost, self.nodes.highestCost)
	self:debug('Iterations %d, expansions %d', self.iterations, self.expansions)
	table.insert(self.path, 1, goal)
	while currentNode.pred and #self.path < 1000 and currentNode ~= currentNode.pred do
		table.insert(self.path, 1, currentNode.pred)
		currentNode = currentNode.pred
	end
end

function HybridAStar:printOpenList(openList)
	print('--- Open list ----')
	for i, node in ipairs(openList.values) do
		print(node)
		if i > 5 then break end
	end
	print('--- Open list end ----')
end

--- A pathfinder combining the (slow) hybrid A * and the (fast) regular A * star.
--- Near the start and the goal the hybrid A * is used to ensure the generated path is drivable (direction changes 
--- always obey the turn radius), but use the A * between the two.
--- We'll run 3 pathfindings: one A * between start and goal (phase 1), then trim the ends of the result in hybridRange
--- Now run a hybrid A * from the start to the beginning of the trimmed A * path (phase 2), then another hybrid A * from the
--- end of the trimmed A * to the goal (phase 3).
HybridAStarWithAStarInTheMiddle = CpObject(Pathfinder)

---@param hybridRange number range in meters around start/goal to use hybrid A *  
function HybridAStarWithAStarInTheMiddle:init(hybridRange)
	-- path generation phases
	self.START_TO_MIDDLE = 1
	self.MIDDLE = 2
	self.MIDDLE_TO_END = 3
	self.ALL_HYBRID = 4 -- start and goal close enough, we only need a single phase with hybrid
	self.hybridRange = hybridRange
end

---@param start State3D start node
---@param goal State3D goal node
---@param turnRadius number turn radius of the vehicle
---@param allowReverse boolean allow reverse driving
---@param getNodePenaltyFunc function get penalty for a node, see getNodePenalty()
---@param fieldPolygon Polygon the field definition polygon
---@param getNodePenaltyFunc function function to calculate the penalty for a node. Typically you want to penalize
--- off-field locations and locations with fruit on the field.
function HybridAStarWithAStarInTheMiddle:start(start, goal, turnRadius, allowReverse, fieldPolygon, getNodePenaltyFunc)
	self.hybridAStarPathFinder = HybridAStar()
	self.aStarPathFinder = Pathfinder()
	self.retries = 0
	self.startNode, self.goalNode, self.turnRadius, self.withReverse = start, goal, turnRadius, allowReverse
	self.fieldPolygon, self.getNodePenaltyFunc = fieldPolygon, getNodePenaltyFunc
	self.hybridRange = self.hybridRange and self.hybridRange or turnRadius * 3
	-- how far is start/goal apart?
	self.startNode:updateH(self.goalNode)
	-- do we even need to use the normal A star or the nodes are close enough that the hybrid A star will be fast enough?
	if self.startNode:getCost() < self.hybridRange * 3 then
		self.phase = self.ALL_HYBRID
		self.coroutine = coroutine.create(self.hybridAStarPathFinder.findPath)
		self.currentPathFinder = self.hybridAStarPathFinder
		return self:resume(start, goal, turnRadius, allowReverse, getNodePenaltyFunc)
	else
		self.phase = self.MIDDLE
		self.coroutine = coroutine.create(self.aStarPathFinder.run)
		self.currentPathFinder = self.aStarPathFinder
		return self:resume(start, goal, fieldPolygon)
	end

	if not self.coroutine then
	end
	return self:resume()
end

function HybridAStarWithAStarInTheMiddle:resume(...)
	local ok, done, path = coroutine.resume(self.coroutine, self.currentPathFinder, ...)
	if not ok then
		self.coroutine = nil
		print(done)
		return true, nil
	end
	if done then
		self.coroutine = nil
		self.nodes = self.hybridAStarPathFinder.nodes
		if self.phase == self.ALL_HYBRID then
			-- start and node near, just one phase, all hybrid, we are done
			return true, path
		elseif self.phase == self.MIDDLE then
			if not path then return true, nil end
			-- middle part ready, now trim start and end to make room for the hybrid parts
			self.middlePath = path
			self.middlePath:shortenStart(self.hybridRange)
			self.middlePath:shortenEnd(self.hybridRange)
			self.phase = self.START_TO_MIDDLE
			-- generate a hybrid part from the start to the middle section's start
			self.coroutine = coroutine.create(self.hybridAStarPathFinder.findPath)
			self.currentPathFinder = self.hybridAStarPathFinder
			local goal = State3D(self.middlePath[1].x, self.middlePath[1].y, self.middlePath[1].nextEdge.angle)
			local x, y, t = HybridAStar.NodeList(1, 5):getNodeIndexes(goal)
			return self:resume(self.startNode, goal, self.turnRadius, self.allowReverse, self.getNodePenaltyFunc)
		elseif self.phase == self.START_TO_MIDDLE then
			-- start and middle sections ready
			self.path = Polygon:new(path)
			-- append middle to start
			-- but remove last point from start as it ovlerlaps with the first in the middle
			table.remove(self.path)
			for _, n in ipairs(self.middlePath) do
				table.insert(self.path, n)
			end
			self.path:calculateData()
			-- generate middle to end
			return self:startMiddleToEnd()
		else
			if path then
				table.remove(self.path)
				for _, n in ipairs(path) do
					table.insert(self.path, n)
				end
				self.path:calculateData()
			end
			return true, self.path
		end
	end
	return false
end

function HybridAStarWithAStarInTheMiddle:startMiddleToEnd()
	self.phase = self.MIDDLE_TO_END
	self.coroutine = coroutine.create(self.hybridAStarPathFinder.findPath)
	self.currentPathFinder = self.hybridAStarPathFinder
	local start = State3D(self.middlePath[#self.middlePath].x, self.middlePath[#self.middlePath].y, self.middlePath[#self.middlePath].prevEdge.angle)
	return self:resume(start, self.goalNode, self.turnRadius, self.allowReverse, self.getNodePenaltyFunc)
end