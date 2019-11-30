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
--- To test the pathfinding:
--- 1. mark the start location/heading with Alt + <
--- 2. mark the goal location/heading with Alt + >
--- 3. watch the path generated ...
DevHelper = CpObject()

function DevHelper:init()
    self.data = {}
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
    self.data.yRotDeg = math.deg(self.yRot)
    self.data.x, self.data.y, self.data.z = getWorldTranslation(node)
    self.data.fieldNum = courseplay.fields:getFieldNumForPosition(self.data.x, self.data.z)
    self.data.hasFruit, _, self.fruitArea, self.totalFruitArea = courseplay:areaHasFruit( self.data.x, self.data.z, nil, 2, 2)
    self.data.fruitAreaPercent = 100 * self.fruitArea / self.totalFruitArea
    self.data.isField, self.fieldArea, self.totalFieldArea = courseplay:isField(self.data.x, self.data.z, 2, 2)
    self.data.fieldAreaPercent = 100 * self.fieldArea / self.totalFieldArea

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
        self:debug('Start %.1f %.1f %.1f', self.yRot, self.data.yRotDeg, courseGenerator.fromCpAngle(self.data.yRotDeg))

        self.start = State3D(self.data.x, -self.data.z, courseGenerator.fromCpAngle(self.data.yRotDeg))
    elseif bitAND(modifier, Input.MOD_LALT) ~= 0 and isDown and sym == Input.KEY_period then
        self:debug('Goal')
        self.pathfinder = HybridAStarWithAStarInTheMiddle(20)
        self.goal = State3D(self.data.x, -self.data.z, courseGenerator.fromCpAngle(self.data.yRotDeg))
        self:debug('Starting pathfinding between %s and %s', tostring(self.start), tostring(self.goal))
        local done, path = self.pathfinder:start(self.start, self.goal, 5, false)
        if done then
            if path then
                self:loadPath(path)
            else
                self:debug('No path found')
            end
        end
    end
end

function DevHelper:mouseEvent(posX, posY, isDown, isUp, mouseKey)
end

function DevHelper:draw()
    if not CpManager.isDeveloper then return end
    local data = {}
    for key, value in pairs(self.data) do
        table.insert(data, {name = key, value = value})
    end
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
