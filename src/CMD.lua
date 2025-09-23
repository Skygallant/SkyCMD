-- Pre-initialize inputs to avoid nil ops
local pitchInput = 0
local rollInput = 0
local yawInput = 0
local brakeInput = 0
local brakedist = 0
local mass = 0
local distKm = 0
local constructVelocity = {0,0,0}
local velocity = 0
local shipspeed = 0
local altM = 0
local atmo_eco = 0
local atmo_mass = 0
local atmo_s_remain = 0
local space_eco = 0
local space_mass = 0
local space_s_remain = 0
local tele_sysId, tele_bodyId, tele_lat, tele_lon, tele_alt = 0, nil, 0, 0, 0
local tele_name, tele_posStr = "",""
local enviro = ((unit.getAtmosphereDensity() or 0) > 0)
local printDebug = false
local AllowAtmo = false --export: Allow autopilot near planets (<= 400 km from surface)
local FuelForBurn = 40 --export: the amount of fuel required to get up to sufficient speed for space travel (in metric tonnes)
local CollisionClearanceKm = 3 --export: minimum clearance radius from path to obstacles (km)
local StopDist = 9 --km
-- Auto Pilot
local autoAlign = false
local autoBrake = false
-- Auto-align smoothing state
local __alignYawCmd = 0
local __alignPitchCmd = 0

Nav = Navigator.new(system, core, unit)
Nav.axisCommandManager:setupCustomTargetSpeedRanges(axisCommandId.longitudinal, {1000, 5000, 10000, 20000, 30000})
Nav.axisCommandManager:setTargetGroundAltitude(0)

-- Default engine manager target speed ranges are not used
-- Initialize axis modes and zero throttle to prevent unintended thrust
if axisCommandId and axisCommandType then
    -- Use Control Unit APIs directly
    unit.setupAxisCommandProperties(axisCommandId.longitudinal, axisCommandType.byThrottle, nil)
    if axisCommandId.lateral then unit.setupAxisCommandProperties(axisCommandId.lateral, axisCommandType.byThrottle, nil) end
    if axisCommandId.vertical then unit.setupAxisCommandProperties(axisCommandId.vertical, axisCommandType.byThrottle, nil) end

    unit.setAxisCommandValue(axisCommandId.longitudinal, 0)
    if axisCommandId.lateral then unit.setAxisCommandValue(axisCommandId.lateral, 0) end
    if axisCommandId.vertical then unit.setAxisCommandValue(axisCommandId.vertical, 0) end
end
-- Disable any altitude stabilization via the manager (manager not used)

-- Widgets and panels intentionally disabled
unit.hideWidget()
system.showHelper(false)
system.showScreen(false)
construct.setDockingMode(1)

-- Destination state (world position)
local atlas = require('atlas')
local __destPos = nil -- vec3 world
-- Forward declaration (used in __ap_start)
local isWithin400kmOfBody

local function clamp(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

local function v3sub(a, b)
    return vec3((a.x or a[1] or 0) - (b.x or b[1] or 0), (a.y or a[2] or 0) - (b.y or b[2] or 0), (a.z or a[3] or 0) - (b.z or b[3] or 0))
end

-- Conditional printer: only prints when printDebug is true,
-- except always prints COMMAND REJECTED/ACCEPTED
local function dprint(msg)
    if msg == 'COMMAND ACCEPTED' or
       msg == 'COMMAND REJECTED' or
       msg == 'COMMAND COMPLETED' or
       msg == 'OBSTACLE DETECTED' or
       msg == 'EMERGENCY STOP' or
       msg == 'INSUFFICIENT FUEL' or
       msg == 'PRIMED' or
       printDebug then
        system.print(msg)
    end
end

-- Time source: prefer getArkTime (new), fallback to getTime (deprecated)
local function nowTime()
    if system.getArkTime then
        local t = system.getArkTime()
        if type(t) == 'number' then return t end
    end
    if system.getTime then
        local t = system.getTime()
        if type(t) == 'number' then return t end
    end
    return 0
end

-- Return total mass (kg) of space fuel across all linked Space Fuel Tanks
local function getTotalSpaceFuelKg()
    local total = 0
    local seen = {}
    if core and core.getElementIdList and (core.getElementDisplayNameById or core.getElementTypeById) then
        local okIds, ids = pcall(core.getElementIdList)
        if okIds and type(ids) == 'table' then
            for _, id in pairs(ids) do
                if not seen[id] then
                    local typeName = nil
                    if core.getElementDisplayNameById then
                        local okDisp, dispName = pcall(core.getElementDisplayNameById, id)
                        if okDisp then typeName = dispName end
                    elseif core.getElementTypeById then
                        local okType, typName = pcall(core.getElementTypeById, id)
                        if okType then typeName = typName end
                    end
                    if type(typeName) == 'string' then
                        local lname = typeName:lower()
                        if lname:find('space') and lname:find('fuel') and lname:find('tank') then
                            -- Estimate content mass from element mass minus dry mass (from item data)
                            local contentKg = 0
                            local okMass, elemMass = false, nil
                            if core.getElementMassById then
                                okMass, elemMass = pcall(core.getElementMassById, id)
                            end
                            local dryKg = 0
                            if core.getElementItemIdById and system.getItem then
                                local okItemId, itemId = pcall(core.getElementItemIdById, id)
                                if okItemId and itemId then
                                    local okItem, item = pcall(system.getItem, itemId)
                                    if okItem and type(item) == 'table' then
                                        dryKg = tonumber(item.unitMass or item.mass or 0) or 0
                                    end
                                end
                            end
                            if okMass and type(elemMass) == 'number' then
                                contentKg = math.max(0, elemMass - (dryKg or 0))
                            end
                            total = total + (contentKg or 0)
                        end
                    end
                end
            end
        end
    end

    return total
end

-- Return true if at least the given mass (t) of space fuel is available
local function hasMinSpaceFuelKg(minT)
    local minKg = minT * 1000
    local totalKg = 0
    local ok, res = pcall(getTotalSpaceFuelKg)
    if ok and type(res) == 'number' then totalKg = res end
    return totalKg >= (minKg or 0)
end

-- Collision checking helpers (use optional |scope| API if present)

local function getObstacleWorldPositions()
    local positions = {}
    local sc = _G.scope or scope -- allow either global reference
    if not sc then return positions end
    -- Support both possible spellings per user note
    local getIds = sc.getConstrcutIds or sc.getConstructIds
    local getPos = sc.getConstrcutWorldPos or sc.getConstructWorldPos
    if type(getIds) ~= 'function' or type(getPos) ~= 'function' then return positions end
    local okIds, ids = pcall(getIds)
    if not okIds or type(ids) ~= 'table' then return positions end
    for _, id in pairs(ids) do
        local okP, p = pcall(getPos, id)
        if okP and p then
            local x = p.x or p[1]; local y = p.y or p[2]; local z = p.z or p[3]
            if x and y and z then table.insert(positions, vec3(x, y, z)) end
        end
    end
    return positions
end

-- Return true if any obstacle lies within a cylinder of radius |clearanceM| along segment P0->P1
local function hasObstacleAlongPath(P0, P1, clearanceM)
    if not P0 or not P1 then return false end
    local obstacles = getObstacleWorldPositions()
    if #obstacles == 0 then return false end
    local seg = v3sub(P1, P0)
    local segLen2 = seg:dot(seg)
    if segLen2 <= 1e-6 then return false end
    local r2 = (clearanceM or 0)^2
    for _, O in ipairs(obstacles) do
        local OP0 = v3sub(O, P0)
        local t = OP0:dot(seg) / segLen2
        if t >= 0 and t <= 1 then
            local closest = P0 + seg * t
            local d2 = v3sub(O, closest):dot(v3sub(O, closest))
            if d2 <= r2 then return true end
        end
    end
    return false
end

-- Autopilot state for destination engagement
local __ap_active = false
local __ap_phase = 'idle' -- 'idle'|'align'|'cruise'|'brake'|'done'
local __ap_startedAt = 0
local __ap_alignHoldSince = 0
local __ap_initialDistM = 0
local __ap_halfDistM = 0
local __ap_lastSpeed = 0
local __ap_speedStableSince = 0
local __ap_maxSpeedGuess = 0 -- best-effort target speed if API available
-- Emergency all-stop state
local __emg_allstop_active = false
-- Warp handling during AP
local __ap_warpTriggered = false
local __ap_warpCheckCooldown = 0.5
local __ap_lastWarpCheck = 0
-- Stop rotation flag when coming to a stop
local __stopRot = false
-- Gentle approach params (space only)
local __ap_approachSpeedKmh = 250
local __ap_approachStopKm = 0


-- Return true if a world position is outside all atmospheres in the current dest system
local function isWorldInSpace(systemId, worldPos)
    local sys = atlas[systemId]
    if not sys or not worldPos then return true end
    local nearestD2, nearestAtmoR = math.huge, 0
    for _, body in pairs(sys) do
        local c = body and body.center
        if c then
            local d = v3sub(worldPos, c)
            local d2 = d:dot(d)
            if d2 < nearestD2 then
                nearestD2 = d2
                nearestAtmoR = (body.hasAtmosphere and body.atmosphereRadius) or (body.radius or 0)
            end
        end
    end
    if nearestD2 == math.huge then return true end
    return math.sqrt(nearestD2) > (nearestAtmoR + 1.0)
end

local function __ap_reset()
    __ap_active = false
    __ap_phase = 'idle'
    __ap_startedAt = 0
    __ap_alignHoldSince = 0
    __ap_initialDistM = 0
    __ap_halfDistM = 0
    __ap_lastSpeed = 0
    __ap_speedStableSince = 0
    __ap_maxSpeedGuess = 0
    __ap_warpTriggered = false
    __ap_lastWarpCheck = 0
    __stopRot = false
end

-- Best-effort helper to force throttle in byThrottle mode (value in [0,1])
local function __ap_setThrottle01(val)
    local lon = val or 0
    -- Prefer using the axis command manager as requested
    if lon >= 0.5 then
        -- Increase throttle (simulate action start)
        Nav.axisCommandManager:updateCommandFromActionStart(axisCommandId.longitudinal, 10000)
        -- Optional: reflect in the cockpit widget
        unit.setAxisCommandValue(axisCommandId.longitudinal, 1)
    else
        -- Cut throttle completely
        Nav.axisCommandManager:resetCommand(axisCommandId.longitudinal)
        unit.setAxisCommandValue(axisCommandId.longitudinal, 0)
    end
    -- Ensure no unintended lateral/vertical throttle
    if axisCommandId.lateral then unit.setAxisCommandValue(axisCommandId.lateral, 0) end
    if axisCommandId.vertical then unit.setAxisCommandValue(axisCommandId.vertical, 0) end
end

local function __ap_start()
    if not __destPos then return end
    if enviro then
        dprint('Autopilot: not starting in atmosphere')
        return
    end
    -- Pre-launch collision check: reject if any obstacle lies along the path
    do
        local P0 = vec3(construct.getWorldPosition())
        local clr = (CollisionClearanceKm or 0) * 1000
        if hasObstacleAlongPath(P0, __destPos, clr) then
            dprint('COMMAND REJECTED')
            dprint('Obstacle detected along path')
            return
        end
    end

    -- Proximity safety: reject destinations near planets when AllowAtmo=false
    if not AllowAtmo then
        if tele_sysId and isWithin400kmOfBody(tele_sysId, __destPos) then
            dprint('COMMAND REJECTED')
            return
        end
    end

    -- Distance snapshot
    local pos = vec3(construct.getWorldPosition())
    __ap_initialDistM = v3sub(__destPos, pos):len()
    __ap_halfDistM = __ap_initialDistM * 0.6
    -- If destination is already very close in space, skip main burn and go straight to gentle approach
    if __ap_initialDistM <= 20000 and tele_sysId and isWorldInSpace(tele_sysId, __destPos) then
        __ap_phase = 'approach_align'
        dprint('Autopilot: short hop (<20 km), gentle approach')
    else
        -- Enforce minimum space fuel requirement (60t)
        if not hasMinSpaceFuelKg(FuelForBurn) then
            dprint('INSUFFICIENT FUEL')
            dprint('Insufficient space fuel (<60t)')
            return
        else
            __ap_phase = 'align'
        end
    end

    
    __ap_active = true
    __ap_startedAt = nowTime()
    __ap_alignHoldSince = 0
    __ap_lastSpeed = 0
    __ap_speedStableSince = 0
    __ap_warpTriggered = false
    __ap_lastWarpCheck = 0

    -- Prepare ship state
    autoAlign = true
    autoBrake = false
    brakeInput = 0

    -- Ensure we are in throttle mode, not cruise/target-speed (for all axes)
    unit.setupAxisCommandProperties(axisCommandId.longitudinal, axisCommandType.byThrottle, nil)
    if axisCommandId.lateral then unit.setupAxisCommandProperties(axisCommandId.lateral, axisCommandType.byThrottle, nil) end
    if axisCommandId.vertical then unit.setupAxisCommandProperties(axisCommandId.vertical, axisCommandType.byThrottle, nil) end

    system.setWaypoint(tele_posStr,false)

    dprint('COMMAND ACCEPTED')
    dprint('Autopilot: align to destination')
end

local function latLonAltToWorld(center, radius, latDeg, lonDeg, altM)
    local lat = math.rad(latDeg or 0)
    local lon = math.rad(lonDeg or 0)
    local rad = (radius or 0) + (altM or 0)
    local x = rad * math.cos(lat) * math.cos(lon)
    local y = rad * math.cos(lat) * math.sin(lon)
    local z = rad * math.sin(lat)
    return vec3((center[1] or 0) + x, (center[2] or 0) + y, (center[3] or 0) + z)
end

local function getBodyCenter(systemId, bodyId)
    local sys = atlas[systemId]
    if not sys then return nil end

    -- Treat bodyId 0 as a valid "space" position around the system barycenter.
    -- Use world origin with zero radius so lat/lon/alt compute to a point in space.
    if bodyId == 0 then
        return {0, 0, 0}, 0
    end

    local body = sys[bodyId]
    if not body then return nil end
    local c = body.center
    local r = body.radius or 0
    return c, r
end

-- Convert a ::pos triple to world coordinates.
-- For bodyId == 0, interpret the triple as world X/Y/Z (space coords).
-- For other bodies, interpret as lat/lon/alt on the given body.
local function posToWorld(systemId, bodyId, a, b, c)
    if bodyId == 0 then
        return vec3(a or 0, b or 0, c or 0)
    end
    local center, radius = getBodyCenter(systemId, bodyId)
    if center and radius then
        return latLonAltToWorld(center, radius, a, b, c)
    end
    return nil
end

-- Find the nearest body's display name within a system to a world position
local function getNearestBodyName(systemId, worldPos)
    local sys = atlas[systemId]
    if not sys or not worldPos then return nil end
    local bestName, bestD2 = nil, nil
    for id, body in pairs(sys) do
        local c = body and body.center
        local n = body and body.name and body.name[1]
        if c and n then
            local d = v3sub(worldPos, c)
            local d2 = d:dot(d)
            if not bestD2 or d2 < bestD2 then
                bestD2 = d2
                bestName = n
            end
        end
    end
    return bestName
end

-- Find a body by its primary display name (case-insensitive) across all systems
local function findBodyByName(name)
    if not name or type(name) ~= 'string' then return nil end
    local needle = name:match('^%s*(.-)%s*$'):lower()
    if needle == '' then return nil end
    for sysId, sys in pairs(atlas) do
        if type(sys) == 'table' then
            for bodyId, body in pairs(sys) do
                local n = body and body.name and body.name[1]
                if type(n) == 'string' and n:lower() == needle then
                    return sysId, bodyId, body
                end
            end
        end
    end
    return nil
end


-- Return true if a world position is within 400 km of any body surface in the given system
isWithin400kmOfBody = function(systemId, worldPos)
    local sys = atlas[systemId]
    if not sys or not worldPos then return false end
    local world = vec3(worldPos.x or worldPos[1], worldPos.y or worldPos[2], worldPos.z or worldPos[3])
    for _, body in pairs(sys) do
        local c = body and body.center
        local R = body and body.radius
        if c and R and R > 0 then
            local center = vec3(c[1], c[2], c[3])
            local surfDist = (world - center):len() - R
            if surfDist <= 400000 then
                return true
            end
        end
    end
    return false
end

-- Distance from craft to destination body’s atmosphere along the line to __destPos (meters)
local function distanceToDestAtmosphere()
    if not __destPos or not tele_sysId then return math.huge end
    if tele_bodyId == 0 then return math.huge end
    local body = (atlas[tele_sysId] or {})[tele_bodyId]
    if not body then return math.huge end
    local R = (body.hasAtmosphere and body.atmosphereRadius) or (body.radius or 0)
    local Ctbl = body.center
    if not (R and R > 0 and Ctbl) then return math.huge end

    local C  = vec3(Ctbl[1], Ctbl[2], Ctbl[3])
    local P0 = vec3(construct.getWorldPosition())
    local dirVec = v3sub(__destPos, P0)
    local dirLen = dirVec:len()
    if dirLen <= 1e-6 then return math.huge end
    local dir = dirVec / dirLen

    -- Ray-sphere intersection: ||(P0 + t*dir) - C||^2 = R^2
    local OC   = v3sub(P0, C)
    local b    = OC:dot(dir)
    local c    = OC:dot(OC) - R * R
    local disc = b * b - c
    if disc and disc >= 0 then
        local root = math.sqrt(disc)
        local t1 = -b - root
        local t2 = -b + root
        local t
        if t1 >= 0 then t = t1
        elseif t2 >= 0 then t = t2 end
        if t and t >= 0 then return t end
    end
    -- Fallback: radial gap to the atmosphere shell
    local dCenter = v3sub(P0, C):len()
    return math.max(0, dCenter - R)
end

-- Parse ::pos{system, body, latDeg, lonDeg, altM}
local function parsePosString(s)
    if type(s) ~= 'string' then return nil end
    local a,b,lat,lon,alt = s:match('::pos{%s*([%-%d%.]+)%s*,%s*([%-%d%.]+)%s*,%s*([%-%d%.]+)%s*,%s*([%-%d%.]+)%s*,%s*([%-%d%.]+)%s*}')
    if not a then return nil end
    return tonumber(a), tonumber(b), tonumber(lat), tonumber(lon), tonumber(alt)
end

-- Handle chat input to set destination
system:onEvent('onInputText', function(self, text)
    if type(text) ~= 'string' then return end

    -- Warp command: user-triggered validation and initiate
    do
        local trimmed = text:match('^%s*(.-)%s*$')
        if trimmed and trimmed:lower() == 'warp' then
            -- Only allow warp when AllowAtmo is true and warp drive is ready
            if not AllowAtmo then
                dprint('COMMAND REJECTED')
                return
            end
            local ready = false
            if warp and warp.getStatus then
                local ok, st = pcall(warp.getStatus)
                if ok and st == 15 then ready = true end
            end
            if ready and warp and warp.initiate then
                dprint('COMMAND ACCEPTED')
                dprint('Warp: initiating (user command)')
                pcall(function() warp.initiate() end)
                __ap_warpTriggered = true
            else
                dprint('COMMAND REJECTED')
            end
            return
        end
    end

    -- Emergency: allstop
    do
        local trimmed = text:match('^%s*(.-)%s*$')
        if trimmed and trimmed:lower() == 'allstop' then
            __emg_allstop_active = true
            autoAlign = false
            autoBrake = false
            brakeInput = 1
            __ap_setThrottle01(0)
            __ap_reset()
            dprint('All stop: throttle 0 and brakes applied')
            dprint('EMERGENCY STOP')
            return
        end
    end

    -- 0) List saved valid waypoints
    do
        local trimmed = text:match('^%s*(.-)%s*$')
        if trimmed and trimmed:lower() == 'list' then
            if not disk then
                dprint('No databank |disk| linked: cannot list')
                return
            end

            -- collect keys from databank using any available API
            local keys = {}
            if disk.getKeyList then
                local kl = disk.getKeyList()
                if type(kl) == 'table' then
                    for _, k in ipairs(kl) do table.insert(keys, tostring(k)) end
                end
            end
            if #keys == 0 and disk.getNbKeys and disk.getKey then
                local n = tonumber(disk.getNbKeys()) or 0
                -- try 0-based then 1-based just in case
                for i = 0, math.max(n - 1, 0) do
                    local k = disk.getKey(i)
                    if k then table.insert(keys, tostring(k)) end
                end
                if #keys == 0 then
                    for i = 1, n do
                        local k = disk.getKey(i)
                        if k then table.insert(keys, tostring(k)) end
                    end
                end
            end

            if #keys == 0 then
                dprint('No keys found in databank')
                return
            end

            local listed = 0
            for _, key in ipairs(keys) do
                local val = disk.getStringValue and disk.getStringValue(key) or nil
                if type(val) == 'string' and val:find('^::pos%{') then
                    local sid, bid, lat, lon, alt = parsePosString(val)
                    if sid then
                        local center, radius = getBodyCenter(sid, bid)
                        if center and radius then
                            dprint(key)
                            listed = listed + 1
                        end
                    end
                end
            end

            if listed == 0 then
                dprint('No valid waypoint entries found')
            else
                dprint(string.format('Listed %d valid waypoint entries', listed))
            end
            return
        end
    end

    -- 1) Named save: 'name ::pos{...}'
    local name, posStr = text:match('^%s*([^:][%w%._%-%s]*)%s+(::pos%b{})%s*$')
    if name and posStr then
        name = name:match('^%s*(.-)%s*$') -- trim
        tele_name, tele_posStr = name, posStr
        if disk and disk.setStringValue then
            disk.setStringValue(tele_name, tele_posStr)
            dprint(string.format('Saved %s -> %s', tele_name, tele_posStr))
        else
            dprint('No databank |disk| linked: cannot save')
        end
        -- Also set as current destination
        tele_sysId, tele_bodyId, tele_lat, tele_lon, tele_alt = parsePosString(tele_posStr)
        if tele_sysId then
            local world = posToWorld(tele_sysId, tele_bodyId, tele_lat, tele_lon, tele_alt)
            if world then
                __destPos = world
                dprint(string.format('Destination set to %s', tele_name))
                -- Start autopilot sequence when setting a new named destination
                __ap_start()
            end
        end
        return
    end

    -- 2) Raw ::pos string
    do
        local sysId, bodyId, lat, lon, alt = parsePosString(text)
        if sysId then
            tele_sysId, tele_bodyId, tele_lat, tele_lon, tele_alt = sysId, bodyId, lat, lon, alt
            tele_name = "Unknown"
            tele_posStr = text
            local world = posToWorld(sysId, bodyId, lat, lon, alt)
            if world then
                __destPos = world
                if bodyId == 0 then
                    dprint(string.format('Destination set to space (x %.1f, y %.1f, z %.1f)', lat, lon, alt))
                else
                    dprint(string.format('Destination set to system %d body %d (lat %.4f, lon %.4f, alt %.1f)', sysId, bodyId, lat, lon, alt))
                end
                -- Start autopilot sequence on new destination
                __ap_start()
                return
            else
                dprint('Invalid ::pos body/system for atlas')
                return
            end
        end
    end

    -- 2.5) Body name directly (e.g., "Alioth")
    do
        local key = text:match('^%s*(.-)%s*$')
        if key and key ~= '' then
            local sysId, bodyId, body = findBodyByName(key)
            if sysId and bodyId and body and body.center then
                tele_name = body.name and body.name[1] or key
                tele_sysId, tele_bodyId = sysId, bodyId
                tele_lat, tele_lon, tele_alt = 0, 0, 0
                -- Set destination to the body center; braking logic will stop near atmosphere
                local c = body.center
                __destPos = vec3(c[1], c[2], c[3])
                dprint(string.format("Destination set to body '%s' (system %s, id %s)", tostring(tele_name), tostring(sysId), tostring(bodyId)))
                __ap_start()
                return
            end
        end
    end

    -- 3) Recall by name: look up in databank
    if disk and disk.getStringValue then
        local key = text:match('^%s*(.-)%s*$')
        local saved = disk.getStringValue(key)
        if saved and saved:find('^::pos%{') then
            tele_name, tele_posStr = key, saved
            local sysId, bodyId, lat, lon, alt = parsePosString(saved)
            if sysId then
                tele_sysId, tele_bodyId, tele_lat, tele_lon, tele_alt = sysId, bodyId, lat, lon, alt
                local world = posToWorld(sysId, bodyId, lat, lon, alt)
                if world then
                    __destPos = world
                    dprint(string.format('Destination recalled: %s -> %s', key, saved))
                    -- Start autopilot sequence on recall
                    __ap_start()
                    return
                else
                    dprint('Saved ::pos not valid for current atlas')
                    return
                end
            end
        end
    end

    dprint('Unrecognized input. Use |name ::pos{...}| to save, |::pos{...}| to set, or a saved name to recall.')
end)

system:onEvent('onFlush', function (self)

    local pitchSpeedFactor = 0.8 -- Pitch axis response factor (>= 0.01)
    local yawSpeedFactor =  1   -- Yaw axis response factor (>= 0.01)
    local rollSpeedFactor = 1.5 -- Roll axis response factor (>= 0.01)

    local brakeSpeedFactor = 3 -- Braking: force scales with speed (>= 0.01)
    local brakeFlatFactor = 1  -- Braking: flat component along velocity (>= 0.01)

    local autoRoll = true      -- [Atmosphere] Return to level when not rolling
    local autoRollFactor = 2   -- [Atmosphere] Auto-roll strength (>= 0.01)

    local turnAssist = true    -- [Atmosphere] Add yaw/pitch to assist turns when rolling
    local turnAssistFactor = 2 -- [Atmosphere] Turn-assist strength (>= 0.01)

    local torqueFactor = 2 -- Force factor applied to reach rotationSpeed<br>(higher value may be unstable)<br>Valid values: Superior or equal to 0.01

    -- validate params
    pitchSpeedFactor = math.max(pitchSpeedFactor, 0.01)
    yawSpeedFactor = math.max(yawSpeedFactor, 0.01)
    rollSpeedFactor = math.max(rollSpeedFactor, 0.01)
    torqueFactor = math.max(torqueFactor, 0.01)
    brakeSpeedFactor = math.max(brakeSpeedFactor, 0.01)
    brakeFlatFactor = math.max(brakeFlatFactor, 0.01)
    autoRollFactor = math.max(autoRollFactor, 0.01)
    turnAssistFactor = math.max(turnAssistFactor, 0.01)

    -- final inputs
    local finalPitchInput = pitchInput + system.getControlDeviceForwardInput()
    local finalRollInput = rollInput + system.getControlDeviceYawInput()
    local finalYawInput = yawInput - system.getControlDeviceLeftRightInput()
    local finalBrakeInput = brakeInput

    -- Axis
    local worldVertical = vec3(core.getWorldVertical()) -- along gravity
    local constructUp = vec3(construct.getWorldOrientationUp())
    local constructForward = vec3(construct.getWorldOrientationForward())
    local constructRight = vec3(construct.getWorldOrientationRight())
    constructVelocity = vec3(construct.getWorldVelocity())
    local constructVelocityDir = vec3(construct.getWorldVelocity()):normalize()
    local currentRollDeg = getRoll(worldVertical, constructForward, constructRight)
    local currentRollDegAbs = math.abs(currentRollDeg)
    local currentRollDegSign = utils.sign(currentRollDeg)

    -- Auto-align: override yaw/pitch to point nose at __destPos (do not override roll)
    do
        if autoAlign and __destPos and not enviro then
            local pos = vec3(construct.getWorldPosition())
            local toDest = v3sub(__destPos, pos)
            local toLen = toDest:len()
            if toLen and toLen > 1e-3 then
                -- Gains (deg -> unit input)
                local yawGain = 0.45
                local pitchGain = 0.45
                local deadzoneDeg = 0.001

                -- Use current axes already fetched
                local fwd = constructForward
                local up = constructUp
                local right = constructRight

                local x = toDest:dot(fwd)
                local yYaw = toDest:dot(right)
                local yPitch = toDest:dot(up)
                local yawDeg = math.deg(math.atan(yYaw, x))
                local pitchDeg = math.deg(math.atan(yPitch, x))

                -- Convert to input with clamp and deadzone
                local function clamp1(v)
                    if v > 1 then return 1 elseif v < -1 then return -1 else return v end
                end

                -- Error shaping: reduce command as we close in (quadratic taper)
                local yawNorm = math.min(math.abs(yawDeg) / 45.0, 1.0)
                local pitchNorm = math.min(math.abs(pitchDeg) / 30.0, 1.0)
                -- Sign conventions: positive yawDeg means target is to the right;
                -- in this script, yaw right corresponds to negative yaw input.
                -- positive pitchDeg means target is above; positive pitch input pitches up.
                local yawCmdRaw = clamp1(-yawGain * yawDeg * yawNorm * yawNorm)
                local pitchCmdRaw = clamp1( pitchGain * pitchDeg * pitchNorm * pitchNorm)
                if math.abs(yawDeg) < deadzoneDeg then yawCmdRaw = 0 end
                if math.abs(pitchDeg) < deadzoneDeg then pitchCmdRaw = 0 end

                -- Command floor: avoid vanishing inputs above the deadzone so alignment completes
                local minCmd = 0.03
                if math.abs(yawDeg) >= deadzoneDeg and yawCmdRaw ~= 0 and math.abs(yawCmdRaw) < minCmd then
                    yawCmdRaw = (yawCmdRaw > 0 and 1 or -1) * minCmd
                end
                if math.abs(pitchDeg) >= deadzoneDeg and pitchCmdRaw ~= 0 and math.abs(pitchCmdRaw) < minCmd then
                    pitchCmdRaw = (pitchCmdRaw > 0 and 1 or -1) * minCmd
                end

                -- Temporal smoothing: exponential moving average
                local alpha = 0.9 -- 0..1, higher = more responsive
                __alignYawCmd = __alignYawCmd + alpha * (yawCmdRaw - __alignYawCmd)
                __alignPitchCmd = __alignPitchCmd + alpha * (pitchCmdRaw - __alignPitchCmd)

                finalYawInput = __alignYawCmd
                finalPitchInput = __alignPitchCmd
            end
        else
            -- Reset smoothing when inactive
            __alignYawCmd = 0
            __alignPitchCmd = 0
        end
    end

    do
        if __destPos then
            local pos = vec3(construct.getWorldPosition())
            distKm = (v3sub(__destPos, pos):len()) / 1000
        end
    end

    -- Rotation
    local constructAngularVelocity = vec3(construct.getWorldAngularVelocity())
    -- If we're coming to a stop, zero out manual/auto align inputs
    if __stopRot then
        finalPitchInput = 0
        finalRollInput = 0
        finalYawInput = 0
    end
    local targetAngularVelocity = finalPitchInput * pitchSpeedFactor * constructRight
                                    + finalRollInput * rollSpeedFactor * constructForward
                                    + finalYawInput * yawSpeedFactor * constructUp

    -- In atmosphere?
    if worldVertical:len() > 0.01 and unit.getAtmosphereDensity() > 0.0 then
        local autoRollRollThreshold = 1.0
        -- autoRoll on AND currentRollDeg is big enough AND player is not rolling
        if autoRoll == true and currentRollDegAbs > autoRollRollThreshold and finalRollInput == 0 then
            local targetRollDeg = utils.clamp(0,currentRollDegAbs-30, currentRollDegAbs+30);  -- we go back to 0 within a certain limit
            if (rollPID == nil) then
                rollPID = pid.new(autoRollFactor * 0.01, 0, autoRollFactor * 0.1) -- magic number tweaked to have a default factor in the 1-10 range
            end
            rollPID:inject(targetRollDeg - currentRollDeg)
            local autoRollInput = rollPID:get()

            targetAngularVelocity = targetAngularVelocity + autoRollInput * constructForward
        end
        local turnAssistRollThreshold = 20.0
        -- turnAssist AND currentRollDeg is big enough AND player is not pitching or yawing
        if turnAssist == true and currentRollDegAbs > turnAssistRollThreshold and finalPitchInput == 0 and finalYawInput == 0 then
            local rollToPitchFactor = turnAssistFactor * 0.1 -- magic number tweaked to have a default factor in the 1-10 range
            local rollToYawFactor = turnAssistFactor * 0.025 -- magic number tweaked to have a default factor in the 1-10 range

            -- rescale (turnAssistRollThreshold -> 180) to (0 -> 180)
            local rescaleRollDegAbs = ((currentRollDegAbs - turnAssistRollThreshold) / (180 - turnAssistRollThreshold)) * 180
            local rollVerticalRatio = 0
            if rescaleRollDegAbs < 90 then
                rollVerticalRatio = rescaleRollDegAbs / 90
            elseif rescaleRollDegAbs < 180 then
                rollVerticalRatio = (180 - rescaleRollDegAbs) / 90
            end

            rollVerticalRatio = rollVerticalRatio * rollVerticalRatio

            local turnAssistYawInput = - currentRollDegSign * rollToYawFactor * (1.0 - rollVerticalRatio)
            local turnAssistPitchInput = rollToPitchFactor * rollVerticalRatio

            targetAngularVelocity = targetAngularVelocity
                                + turnAssistPitchInput * constructRight
                                + turnAssistYawInput * constructUp
        end
    end

    -- Engine commands
    local keepCollinearity = 1 -- for easier reading
    local dontKeepCollinearity = 0 -- for easier reading
    local tolerancePercentToSkipOtherPriorities = 1 -- if we are within this tolerance (in%), we don't go to the next priorities

    -- Rotation
    if __stopRot then
        -- Force target rotational speed to zero to damp rotation
        targetAngularVelocity = vec3()
    end
    local angularAcceleration = torqueFactor * (targetAngularVelocity - constructAngularVelocity)
    local airAcceleration = vec3(construct.getWorldAirFrictionAngularAcceleration())
    angularAcceleration = angularAcceleration - airAcceleration -- Try to compensate air friction
    Nav:setEngineTorqueCommand('torque', angularAcceleration, keepCollinearity, 'airfoil', '', '', tolerancePercentToSkipOtherPriorities)

    -- Brakes
    local brakeAcceleration = -finalBrakeInput * (brakeSpeedFactor * constructVelocity + brakeFlatFactor * constructVelocityDir)
    Nav:setEngineForceCommand('brake', brakeAcceleration)

    -- AutoNavigation regroups all the axis command by 'TargetSpeed'
    local autoNavigationEngineTags = ''
    local autoNavigationAcceleration = vec3()
    local autoNavigationUseBrake = false

    -- Longitudinal Translation
    local longitudinalEngineTags = 'thrust analog longitudinal'
    local longitudinalCommandType = Nav.axisCommandManager:getAxisCommandType(axisCommandId.longitudinal)
    if (longitudinalCommandType == axisCommandType.byThrottle) then
        local longitudinalAcceleration = Nav.axisCommandManager:composeAxisAccelerationFromThrottle(longitudinalEngineTags,axisCommandId.longitudinal)
        Nav:setEngineForceCommand(longitudinalEngineTags, longitudinalAcceleration, keepCollinearity)
    elseif  (longitudinalCommandType == axisCommandType.byTargetSpeed) then
        -- Force byThrottle composition; do not use target-speed mode
        unit.setupAxisCommandProperties(axisCommandId.longitudinal, axisCommandType.byThrottle, nil)
        local longitudinalAcceleration = Nav.axisCommandManager:composeAxisAccelerationFromThrottle(longitudinalEngineTags,axisCommandId.longitudinal)
        Nav:setEngineForceCommand(longitudinalEngineTags, longitudinalAcceleration, keepCollinearity)
    end

    -- Lateral Translation
    local lateralStrafeEngineTags = 'thrust analog lateral'
    local lateralCommandType = Nav.axisCommandManager:getAxisCommandType(axisCommandId.lateral)
    if (lateralCommandType == axisCommandType.byThrottle) then
        local lateralStrafeAcceleration =  Nav.axisCommandManager:composeAxisAccelerationFromThrottle(lateralStrafeEngineTags,axisCommandId.lateral)
        Nav:setEngineForceCommand(lateralStrafeEngineTags, lateralStrafeAcceleration, keepCollinearity)
    elseif  (lateralCommandType == axisCommandType.byTargetSpeed) then
        unit.setupAxisCommandProperties(axisCommandId.lateral, axisCommandType.byThrottle, nil)
        local lateralStrafeAcceleration =  Nav.axisCommandManager:composeAxisAccelerationFromThrottle(lateralStrafeEngineTags,axisCommandId.lateral)
        Nav:setEngineForceCommand(lateralStrafeEngineTags, lateralStrafeAcceleration, keepCollinearity)
    end

    -- Vertical Translation
    local verticalStrafeEngineTags = 'thrust analog vertical'
    local verticalCommandType = Nav.axisCommandManager:getAxisCommandType(axisCommandId.vertical)
    if (verticalCommandType == axisCommandType.byThrottle) then
        local verticalStrafeAcceleration = Nav.axisCommandManager:composeAxisAccelerationFromThrottle(verticalStrafeEngineTags,axisCommandId.vertical)
        Nav:setEngineForceCommand(verticalStrafeEngineTags, verticalStrafeAcceleration, keepCollinearity, 'airfoil', 'ground', '', tolerancePercentToSkipOtherPriorities)
    elseif  (verticalCommandType == axisCommandType.byTargetSpeed) then
        unit.setupAxisCommandProperties(axisCommandId.vertical, axisCommandType.byThrottle, nil)
        local verticalStrafeAcceleration = Nav.axisCommandManager:composeAxisAccelerationFromThrottle(verticalStrafeEngineTags,axisCommandId.vertical)
        Nav:setEngineForceCommand(verticalStrafeEngineTags, verticalStrafeAcceleration, keepCollinearity, 'airfoil', 'ground', '', tolerancePercentToSkipOtherPriorities)
    end

    -- Auto Navigation (Cruise Control)
    if (autoNavigationAcceleration:len() > constants.epsilon) then
        if (brakeInput ~= 0 or autoNavigationUseBrake or math.abs(constructVelocityDir:dot(constructForward)) < 0.95)  -- if the velocity is not properly aligned with the forward
        then
            autoNavigationEngineTags = autoNavigationEngineTags .. ', brake'
        end
        Nav:setEngineForceCommand(autoNavigationEngineTags, autoNavigationAcceleration, dontKeepCollinearity, '', '', '', tolerancePercentToSkipOtherPriorities)
    end

end)

-- During autopilot, check for newly encountered obstacles and emergency stop if collision likely
system:onEvent('onEnter', function(self)
    if __ap_active and __destPos and not enviro then
        local P0 = vec3(construct.getWorldPosition())
        local clr = (CollisionClearanceKm or 0) * 1000
        if hasObstacleAlongPath(P0, __destPos, clr) then
            __emg_allstop_active = true
            dprint('OBSTACLE DETECTED')
            dprint('EMERGENCY STOP')
        end
    end
end)

system:onEvent('onUpdate', function (self)
    Nav:update()
    enviro = ((unit.getAtmosphereDensity() or 0) > 0)
    local v = vec3(construct.getWorldVelocity())
    shipspeed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
    -- Stopping criteria (forward speed and angular speed)
    local fwdVec = vec3(construct.getWorldOrientationForward())
    local forwardSpeed = math.abs(v:dot(fwdVec))
    local angSpeed = vec3(construct.getWorldAngularVelocity()):len()
    local function fullyStopped()
        return (forwardSpeed <= 0.5) and (angSpeed <= 0.5)
    end
    mass = construct.getTotalMass() or 0
    local function nz(x) return (x ~= nil) and x or 0 end
    local function finite(x) return x == x and x ~= math.huge and x ~= -math.huge end

    local force = construct.getCurrentBrake()
    if force == nil or force <= 0 then force = construct.getMaxBrake() or 0 end

    local vx, vy, vz = nz(constructVelocity and constructVelocity.x),
                    nz(constructVelocity and constructVelocity.y),
                    nz(constructVelocity and constructVelocity.z)

    local s2 = vx*vx + vy*vy + vz*vz
    local brakespeed = (s2 >= 0 and finite(s2)) and math.sqrt(s2) or 0
    brakedist = math.floor(mass * brakespeed * brakespeed) / (2 * force)

    if enviro then
        autoAlign = false
        if autoBrake then brakeInput = 0 end
        autoBrake = false
        if __ap_active then
            dprint('Autopilot: canceled (atmosphere detected)')
            __ap_reset()
        end
        __stopRot = false
    end

    if __destPos then
        if isWorldInSpace(tele_sysId, __destPos) then
            if autoBrake and math.floor(distKm) < math.ceil(brakedist/1000) + StopDist and shipspeed > 10 then --kilometers
                brakeInput = 1
            elseif autoBrake and (shipspeed <= 1) then
                brakeInput = 0
                autoBrake = false
            end
        else
            if autoBrake and math.floor(distanceToDestAtmosphere()) < math.ceil(brakedist+49000) and shipspeed > 10 then --meters
                brakeInput = 1
            elseif autoBrake and (shipspeed <= 1) then
                brakeInput = 0
                autoBrake = false
            end
        end
    end

    -- Autopilot sequencer
    if __ap_active and __destPos and not enviro then
        local now = nowTime()

        -- User-issued warp command handles warp validation; no periodic check here

        -- Helper: angle error (deg) between forward and destination vector
        local function angleToDestDeg()
            local pos = vec3(construct.getWorldPosition())
            local toDest = v3sub(__destPos, pos)
            local L = toDest:len()
            if not L or L <= 1e-6 then return 0 end
            local fwd = vec3(construct.getWorldOrientationForward())
            local cosang = (toDest / L):dot(fwd)
            cosang = math.min(1, math.max(-1, cosang))
            return math.deg(math.acos(cosang))
        end

        -- Current distance to destination (meters)
        local function distanceToDestM()
            local pos = vec3(construct.getWorldPosition())
            return v3sub(__destPos, pos):len()
        end

        if __ap_phase == 'align' then
            autoAlign = true
            Nav.axisCommandManager:updateTargetGroundAltitudeFromActionLoop(1.0)

            -- consider aligned when within tolerance and stable briefly
            local tolDeg = 1.0
            if angleToDestDeg() <= tolDeg then
                if __ap_alignHoldSince == 0 then __ap_alignHoldSince = now end
                if (now - __ap_alignHoldSince) >= 0.5 then
                    -- Accelerate using raw throttle; enable auto-brake
                    __ap_setThrottle01(1)
                    autoAlign = false
                    dprint('Autopilot: aligned; throttle max with auto-brake')
                    __ap_phase = 'cruise'
                    __ap_lastSpeed = shipspeed or 0
                    __ap_speedStableSince = now
                end
            else
                __ap_alignHoldSince = 0
            end

        elseif __ap_phase == 'cruise' then
            -- Detect max speed reached using construct.getMaxSpeed()
            Nav.axisCommandManager:updateTargetGroundAltitudeFromActionLoop(-1.0)
            local maxSpd = (construct.getMaxSpeed and construct.getMaxSpeed()) or 0
            local atMax = false
            if maxSpd and maxSpd > 0 then
                atMax = (shipspeed >= (0.995 * maxSpd))
            else
                -- Fallback: plateau detection
                local eps = 1.0 -- m/s
                if shipspeed > (__ap_lastSpeed + eps) then
                    __ap_lastSpeed = shipspeed
                    __ap_speedStableSince = now
                end
                atMax = ((now - __ap_speedStableSince) >= 3.0)
            end

            local distNow = distanceToDestM()
            local halfReached = (__ap_halfDistM > 0) and (distNow <= __ap_halfDistM)

            if halfReached or atMax then
                -- Begin braking phase: throttle to 0; auto-brake will handle decel near target
                __ap_setThrottle01(0)
                autoBrake = true
                dprint('Autopilot: throttle 0; braking...')
                __ap_phase = 'brake'
            end

        elseif __ap_phase == 'brake' then
            __stopRot = true
            -- When nearly stopped, optionally run gentle approach in space, else exit
            if fullyStopped() then
                if tele_sysId and isWorldInSpace(tele_sysId, __destPos) then
                    __ap_phase = 'approach_align'
                    __ap_alignHoldSince = 0
                    autoAlign = true
                    dprint('Autopilot: gentle approach setup')
                else
                    autoAlign = false
                    __ap_phase = 'done'
                    dprint('Autopilot: arrived; exiting seat')
                    dprint('COMMAND COMPLETED')
                    pcall(function()
                        __ap_setThrottle01(0)
                        unit.exit()
                    end)
                    __ap_reset()
                end
            end

        elseif __ap_phase == 'approach_align' then
            -- Re-align precisely before gentle approach thrust
            autoAlign = true
            Nav.axisCommandManager:updateTargetGroundAltitudeFromActionLoop(1.0)
            local tolDeg = 1.0
            if angleToDestDeg() <= tolDeg then
                if __ap_alignHoldSince == 0 then __ap_alignHoldSince = now end
                if (now - __ap_alignHoldSince) >= 0.3 then
                    -- Do not use target-speed mode; throttle-limited gentle approach
                    __ap_setThrottle01(1) -- engage throttle to start approach
                    __ap_phase = 'approach'
                    autoAlign = false
                    dprint('Autopilot: gentle approach (<=200 km/h) with autobrake')
                end
            else
                __ap_alignHoldSince = 0
            end

        elseif __ap_phase == 'approach' then
            -- Keep aligning lightly and let autobrake stop near the target; cap speed without target-speed
            --autoAlign = true
            Nav.axisCommandManager:updateTargetGroundAltitudeFromActionLoop(-1.0)
            local distNowKm = (distanceToDestM() / 1000)
            if not autoBrake and distNowKm <= (1) and fullyStopped() then
                -- Stop complete: exit
                --autoAlign = false
                __ap_phase = 'done'
                dprint('Autopilot: arrived; exiting seat')
                dprint('COMMAND COMPLETED')
                pcall(function()
                    __ap_setThrottle01(0)
                    unit.exit()
                end)
                __ap_reset()
            end
            if distNowKm <= (2) and shipspeed > 20 then
                __ap_setThrottle01(0)
                StopDist = __ap_approachStopKm -- Stop at this many km from destination
                autoBrake = true
            end
            local cap_ms = (__ap_approachSpeedKmh / 3.6)
            if shipspeed > cap_ms then
                __ap_setThrottle01(0)
                StopDist = __ap_approachStopKm -- Stop at this many km from destination
                autoBrake = true
            end
        else
            __stopRot = false
        end
    end

    -- Emergency all-stop handler
    if __emg_allstop_active then
        __stopRot = true
        __ap_setThrottle01(0)
        autoAlign = false
        brakeInput = 1
        if fullyStopped() then
            brakeInput = 0
            __emg_allstop_active = false
            __stopRot = false
            pcall(function()
                unit.exit()
            end)
        end
    end
end)

dprint("PRIMED")
