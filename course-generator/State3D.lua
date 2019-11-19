
---@class State3D
State3D = CpObject()

---@param x number x position
---@param y number y position
---@param t number heading (theta) in radians
---@param r number turn radius
---@param pred State3D predecessor node
---@param motionPrimitive HybridAstar.MotionPrimitive straight/left/right
function State3D:init(x, y, t, g, pred, motionPrimitive)
    self.x = x
    self.y = y
    self.t = self:normalizeHeadingRad(t)
    self.pred = pred
    self.g = g
    self.h = 0
    self.cost = 0
    self.goal = false
    self.onOpenList = false
    self.open = false
    self.closed = false
    self.motionPrimitive = motionPrimitive
end

--- Comparision for the binary heap to find the node with the lowest cost
function State3D:lt(other)
    return self.cost < other.cost
end

--- Mark this node as the goal
function State3D:setGoal()
    self.goal = true
end

function State3D:isGoal()
    return self.goal
end

function State3D:close()
    self.closed = true
end

function State3D.pop(openList)
    local node = openList:pop()
    node.onOpenList = false
    return node
end

function State3D:insert(openList)
    self.closed = false
    if not self.onOpenList then
        self.onOpenList = true
        openList:insert(self)
    end
end

function State3D:isClosed()
    return self.closed
end

function State3D:equals(other, deltaPos, deltaTheta)
    return math.abs(self.x - other.x ) < deltaPos and
            math.abs(self.y - other.y ) < deltaPos and
            math.abs(self.t - other.t) < deltaTheta
end

function State3D:updateG(primitive)
    local penalty = 1
    local reverseAffinity = 2
    if self.pred and self.pred.motionPrimitive then
        -- penalize turning
        if primitive.type ~= self.pred.motionPrimitive.type then
            penalty = penalty * 1.05
        end
        -- penalize direction change
        if not HybridAStar.MotionPrimitives.isSameDirection(primitive, self.pred.motionPrimitive) then
            penalty = penalty * reverseAffinity * 2
        end
        -- penalize reverse driving
        if HybridAStar.MotionPrimitives.isReverse(primitive) then
            penalty = penalty * reverseAffinity
        end
    end
    self.g = self.g + penalty * primitive.d
end

---@param node State3D
function State3D:updateH(goal)
    -- simple Eucledian heuristics
    local dx = goal.x - self.x
    local dy = goal.y - self.y
    self.h = math.sqrt(dx * dx + dy * dy)
    self.cost = self.g + self.h
end

---@param node State3D
function State3D:updateHWithDubins(goal, turnRadius)
    local dubinsPath = dubins_shortest_path(self, goal, turnRadius)
    local dubinsPathLength = dubins_path_length(dubinsPath)
    --print(self.h, dubinsPathLength)
    self.h = math.max(dubinsPathLength, self.h)
    self.cost = self.g + self.h
end

function State3D:getCost()
    return self.cost
end

function State3D:normalizeHeadingRad(t)
    t = t % (2 * math.pi)
    if t < 0 then
        return 2 * math.pi - t
    else
        return t
    end
end

function State3D:createSuccessor(primitive)
    local xSucc = self.x + primitive.dx * math.cos(self.t) - primitive.dy * math.sin(self.t)
    local ySucc = self.y + primitive.dx * math.sin(self.t) + primitive.dy * math.cos(self.t)
    local tSucc = self:normalizeHeadingRad(self.t + primitive.dt)
    return State3D(xSucc, ySucc, tSucc, self.g, self, primitive)
end

function State3D:__tostring()
    local result
    local type = self.motionPrimitive and tostring(self.motionPrimitive.type) or 'nil'
    result = string.format('x: %.2f y:%.2f t:%d type:%s cost:%.1f closed:%s open:%s', self.x, self.y, math.deg(self.t),
            type, self.cost, tostring(self.closed), tostring(self.open))
    return result
end
