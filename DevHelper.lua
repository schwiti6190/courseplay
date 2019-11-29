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

--- Development helper utilities to easily test and diagnose things.
DevHelper = CpObject()

function DevHelper:init()
end

function DevHelper:debug(...)
    print(string.format(...))
end

function DevHelper:update()
    if not CpManager.isDeveloper then return end
    local node, lx, lz
    if g_currentMission.controlledVehicle then
        node = g_currentMission.controlledVehicle.rootNode
        lx, _, lz = localDirectionToWorld(node, 0, 0, 1)
    else
        node = g_currentMission.player.cameraNode
        lx, _, lz = localDirectionToWorld(node, 0, 0, -1)
    end
    self.yRot = math.atan2( lx, lz )
    self.x, self.y, self.z = getWorldTranslation(node)
    self.fieldNum = courseplay.fields:getFieldNumForPosition(self.x, self.z)
    self.hasFruit = courseplay:areaHasFruit( self.x, self.z, nil, 1)
    self.isField = courseplay:isField(self.x, self.z)

    if self.pathfinder and self.pathfinder:isActive() then
        local done, path = self.pathfinder:resume()
        if done then
            self:loadPath(path)
        end
    end

end

function DevHelper:keyEvent(unicode, sym, modifier, isDown)
    if not CpManager.isDeveloper then return end
    if bitAND(modifier, Input.MOD_LALT) ~= 0 and isDown and sym == Input.KEY_comma then
        self:debug('Start %.1f %.1f %.1f', self.yRot, math.deg(self.yRot), courseGenerator.fromCpAngle(math.deg(self.yRot)))

        self.start = State3D(self.x, -self.z, courseGenerator.fromCpAngle(math.deg(self.yRot)))
    elseif bitAND(modifier, Input.MOD_LALT) ~= 0 and isDown and sym == Input.KEY_period then
        self:debug('Goal')
        self.pathfinder = HybridAStarWithAStarInTheMiddle(20)
        self.goal = State3D(self.x, -self.z, courseGenerator.fromCpAngle(math.deg(self.yRot)))
        self:debug('Starting pathfinding between %s and %s', tostring(self.start), tostring(self.goal))
        local done, path = self.pathfinder:start(self.start, self.goal, 5, false,
                Polygon:new(courseGenerator.pointsToXy(courseplay.fields.fieldData[self.fieldNum].points)), DevHelper.getNodePenalty)
        if done then
            if path then
                self:loadPath(path)
            else
                self:debug('No path found')
            end
        end
    end
end

---@param node State3D
function DevHelper.getNodePenalty(node)
    local penalty = 0
    if not courseplay:isField(node.x, -node.y) then
        penalty = penalty + 200
    end
    if courseplay:areaHasFruit( node.x, -node.y, nil, 1) then
        penalty = penalty + 100
    end
    return penalty
end

function DevHelper:mouseEvent(posX, posY, isDown, isUp, mouseKey)
end

function DevHelper:draw()
    if not CpManager.isDeveloper then return end
    local data = {}
    table.insert(data, {name = "x", value = self.x})
    table.insert(data, {name = "z", value = self.z})
    table.insert(data, {name = "heading", value = math.deg(self.yRot)})
    table.insert(data, {name = "field", value = self.fieldNum})
    table.insert(data, {name = "isField", value = self.isField})
    table.insert(data, {name = "hasFruit", value = self.hasFruit})
    DebugUtil.renderTable(0.2, 0.2, 0.02, data, 0.05)
    self:drawCourse()
end

---@param path State3D[]
function DevHelper:loadPath(path)
    self:debug('Path with %d waypoint found', #path)
    self.course = Course(nil, courseGenerator.pointsToXz(path), true)
end

function DevHelper:drawCourse()
    if not self.course then return end
    for i = 1, self.course:getNumberOfWaypoints() do
        local x, y, z = self.course:getWaypointPosition(i)
        cpDebug:drawPoint(x, y + 3, z, 10, 0, 0)
        Utils.renderTextAtWorldPosition(x, y + 3.2, z, tostring(i), getCorrectTextSize(0.012), 0)
        if i < self.course:getNumberOfWaypoints() then
            local nx, ny, nz = self.course:getWaypointPosition(i + 1)
            cpDebug:drawLine(x, y + 3, z, 0, 0, 100, nx, ny + 3, nz)
        end
    end
end

-- make sure to recreate the global dev helper whenever this script is (re)loaded
g_devHelper = DevHelper()
