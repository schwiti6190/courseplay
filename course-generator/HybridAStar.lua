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

---@class HybridAStar
HybridAStar = CpObject()

--- Motion primitives for node expansions, contains the dx/dy/dt values for
--- driving straight/right/left. The idea is to calculate these once as they are
--- only dependent on the turn radius, and then use the precalculated values during the search.
---@class HybridAstar.MotionPrimitives
HybridAStar.MotionPrimitives = CpObject()
-- forward straight/right/left
HybridAStar.MotionPrimitiveTypes = {FS = 'FS', FR = 'FR', FL = 'FL', RS = 'RS', RR = 'RR', RL = 'RL'}

---@param r number turning radius
---@param expansionDegree number degrees of arc in one expansion step
---@param withReverse boolean allow for reversing
function HybridAStar.MotionPrimitives:init(r, expansionDegree, withReverse)
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
	if withReverse then
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

---@class HybridAStar.closedList
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
	--print('node ', node.x, node.y, node.t)
	--print('index ', x, y, t)
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
	self.gridSpacing = 4
	self.count = 0
	self.yields = 0
	self.fruitToCheck = nil
	self.customHasFruitFunc = nil
	self.iterations = 0
end

function HybridAStar:debug(...)
	print(string.format(...))
end

---@param start State3D start node
---@param goal State3D goal node
function HybridAStar:findPath(start, goal, turnRadius, withReverse)
	local motionPrimitives = HybridAStar.MotionPrimitives(turnRadius, 6.75, withReverse)
	print(motionPrimitives)
	local deltaPos = 0.2
	local deltaTheta = 5

	-- create the open list for the nodes as a binary heap where
	-- the node with the lowest total cost is at the top
	local lt = function(a, b)
		return a:lt(b)
	end
	local openList = BinaryHeap.minHeap(lt)

	-- create the configuration space
	---@type HybridAStar.closedList closedList
	local nodes = HybridAStar.NodeList(1, deltaTheta)

	start:updateH(goal)
	start:insert(openList)
	nodes:add(start)
	goal:setGoal()
	goal:updateH(start)
	nodes:add(goal)
	self.iterations = 0
	self.expansions = 0
	while openList:size() > 0 and self.iterations < 2000000 do
		-- pop lowest cost node from queue
		---@type State3D
		local pred = State3D.pop(openList)
		--print('pred ' .. tostring(pred))
		if not pred:isClosed() then
			local existingPredNode = nodes:get(pred)
			if existingPredNode and existingPredNode:equals(goal, deltaPos, math.rad(deltaTheta)) then
				-- done!
				print('Node popped is the goal node!')
				self.nodes = nodes
				--self:printOpenList(openList)
				self:rollUpPath(existingPredNode)
				return
			end
			-- create the successor nodes
			for _, primitive in ipairs(motionPrimitives:getPrimitives()) do
				---@type State3D
				local succ = pred:createSuccessor(primitive)
				succ:updateG(primitive)
				succ:updateH(goal)
				--succ:updateHWithDubins(goal, turnRadius)
				if succ.h < 2 * turnRadius then
					--succ:updateHWithDubins(goal, turnRadius)
				end
				local existingSuccNode = nodes:get(succ)
				if existingSuccNode then
					if not existingSuccNode:isClosed() then
						if existingSuccNode:equals(goal, deltaPos, math.rad(deltaTheta)) then
							self.nodes = nodes
							existingSuccNode.pred = succ.pred
							--self:printOpenList(openList)
							self:rollUpPath(existingSuccNode)
							return
						end
						if existingSuccNode:getCost() > succ:getCost() then
							--print('replace existing ' .. existingSuccNode.motionPrimitive.type .. ' with ' .. primitive.type)
							-- successor cell already exist but the new one is cheaper, replace
							if nodes:inSameCell(existingSuccNode, existingPredNode) then
								--self:debug('HOOPPP')
								succ.pred = existingPredNode.pred
							end
							succ.open = existingSuccNode.open
							nodes:add(succ)
						end
					end
				else
					if (math.abs(succ.x) < 2000 and math.abs(succ.y) < 2000 ) then
						--print('not existing ' .. primitive.type)
						-- successor cell does not yet exist
						nodes:add(succ)
						-- put it on the open list as well
						succ:insert(openList)
					end
				end
			end
			pred:close()
			self.expansions = self.expansions + 1
		end
		self.iterations = self.iterations + 1
		--print(self.iterations, openList:size())
	end
	self.nodes = nodes
	self:debug('No path found: iterations %d, expansions %d', self.iterations, self.expansions)
	--self.nodes:print()
end

---@param node State3D
function HybridAStar:rollUpPath(node)
	self.path = {}
	local currentNode = node
	self:debug('Goal node at %.1f/%1.f, cost %.1f (%.1f - %.1f', node.x, node.y, node.cost,
			self.nodes.lowestCost, self.nodes.highestCost)
	self:debug('Iterations %d, expansions %d', self.iterations, self.expansions)
	table.insert(self.path, 1, node)
	while currentNode.pred and #self.path < 1000 and currentNode ~= currentNode.pred do
		table.insert(self.path, 1, currentNode.pred)
		currentNode = currentNode.pred
	end

end

function HybridAStar:printOpenList(openList)
	self.nodes:print()
	local i = 1
	while openList:size() > 0 and i < 20 do
		local node = openList:pop()
		print(node.cost)
		i = i + 1
	end
end

