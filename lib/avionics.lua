--[[
Dual Universe Avionics Library

Author: Stochasty
Date: 8/28/2022

Description: The Dual Universe Avionics Library is a library of avionics functions designed to provide improved
in-flight ship controls, such as automatic landing, more responsive pitch/roll/yaw behavior, and
much improved hover engine altitude control.
]]

-- Helper functions and useful constants
avionicsMode = {
    none = -1,
    auto = 0,
    ground = 1,
    atmosphere = 2,
    space = 3,
    orbit = 4,
    landing = 5,
}

avionicsInputType = {
    pitch = 1,
    roll = 2,
    yaw = 3,
    longitudinal = 4,
    lateral = 5,
    vertical = 6,
    ground = 7,
    landing = 8,
    brake = 9,
}

avionicsInputFlag = {
    active = 1,
    inactive = 2,
    locked = 3,
}

function signedRotationAngle(normal, vecA, vecB)
    return math.deg(math.atan(vecA:cross(vecB):dot(normal), vecA:dot(vecB)))
end

function normalizeAngle(theta)  --normalizes an angle (in degrees) to the range [-180..180]
    return (theta + 180) % 360 - 180
end

function reverseLookup(t, v)
    for k, _v in pairs(t) do
        if _v == v then
            return k
        end
    end
    return nil
end

function round(x, d)
    return math.floor(x*10^d + 0.5)/10^d
end

function getLinksByFunctionName(control, fn)
    links = {}
    for k,v in pairs(control) do
        if (type(v) == "table") and (type(v.export) == "table") then
            if v[fn] ~= nil then
                links[v.getLocalId()] = v
            end
        end
    end
    return links
end

-- Avionics Namespace
Avionics = {}
Avionics.__index = Avionics

function Avionics.new(nav, system, core, control, construct)
    local self = setmetatable({}, Avionics)
    self.nav = nav
    self.system = system
    self.core = core
    self.control = control
    self.construct = construct

    self.mass = construct.getMass()
    self.imass = construct.getInertialMass()

    self.controlMode = avionicsMode.none
    self.oldControlMode = avionicsMode.none
    
    self.hoverEngineLinks = getLinksByFunctionName(self.control, 'getDistance')
    self.databaseLinks = getLinksByFunctionName(self.control, 'getKeys')

    self.controlInputs = {
        pitch = 0,
        roll = 0,
        yaw = 0,
        --longitudinal = 0,
        lateral = 0,
        vertical = 0,
        ground = 0,
        landing = 0,
        brake = 0,
    }

    self.controlInputFactors = {
        pitch = {start = 1, loop = 1},
        roll = {start = 1, loop = 1},
        yaw = {start = 1, loop = 1},
        longitudinal = {start = 1, loop = 1},
        lateral = {start = 1, loop = 1},
        vertical = {start = 1, loop = 1},
        ground = {start = 1, loop = 1},
        landing = {start = 0, loop = 0},
        brake = {start = 1, loop = 1},
    }

    self.controlAccelerationFactors = {
        pitch = 1,
        roll = 1,
        yaw = 1,
        --longitudinal = 1,
        lateral = 1,
        vertical = 1,
        ground = 0.1,
        landing = 0,
        brake = {flat = 10, linear = 10}
    }

    self.controlFlags = {
        pitch = avionicsInputFlag.inactive,
        roll = avionicsInputFlag.inactive,
        yaw = avionicsInputFlag.inactive,
        --longitudinal = 1,
        lateral = avionicsInputFlag.inactive,
        vertical = avionicsInputFlag.inactive,
        ground = avionicsInputFlag.inactive,
        landing = avionicsInputFlag.inactive,
        brake = avionicsInputFlag.inactive,
    }

    self.targetPitchDeg = 0
    self.targetRollDeg = 0
    self.maxPitchDeg = 30
    self.maxRollDeg = 30
    --self.targetHeading = 0
    self.braking = false
    self.autoBrakeThreshold = -0.1

    --self.hoverAltitude = self.core.getAltitude() + 1
    --self.hoverAmortizationFactor = 0.99
    --self.hoverVelocityFactor = 1
    self.minGroundDistance = 10
    self.maxGroundDistance = self.control.computeGroundEngineAltitudeStabilizationCapabilities()[2]
    self.landingSpeed = 50/3.6
    self.landingDistance = 50

    self.pid = {
        pitch = pid.new(5, 0, 0),
        roll = pid.new(5, 0, 0),
        yaw = pid.new(5, 0, 0),
        --longitudinal = pid.new(0.2, 0, 10),
        lateral = pid.new(1, 0, 0),
        vertical = pid.new(1, 0, 0),
        --vertical = pid.new(0.2, 0, 10),
        ground = pid.new(1, 0, 5),
        brake = pid.new(1, 0, 0),
    }

    self:getConstructFlightParameters()

    return self
end

function Avionics.setControlMode(self, controlMode)
    self.controlMode = controlMode
    if controlMode ~= avionicsMode.none then
        for k, _ in pairs(self.controlFlags) do
            self.controlFlags[k] = avionicsInputFlag.inactive
        end
    end
end

function Avionics.getControlMode(self)
    local controlMode = avionicsMode.none
    if self.controlMode == avionicsMode.auto then
        controlMode = self:getAutoControlMode()
    else
        controlMode = self.controlMode
    end
    return controlMode
end

function Avionics.getAutoControlMode(self)
    local controlMode = avionicsMode.none
    if (self.worldVertical:len() > 0.01) and ((self.control.getAtmosphereDensity() > 0.0) or ((self.currentAltitude < 1000) and (self.currentAltitude ~= 0))) then
        controlMode = avionicsMode.atmosphere
    else
        controlMode = avionicsMode.space
    end
    return controlMode
end

function Avionics.getConstructFlightParameters(self)
    self.worldVertical = -vec3(self.core.getWorldVertical())

    self.constructUp = vec3(self.construct.getWorldOrientationUp())
    self.constructForward = vec3(self.construct.getWorldOrientationForward())
    self.constructRight = vec3(self.construct.getWorldOrientationRight())

    self.roll = getRoll(self.worldVertical, self.constructForward, -self.constructRight)
    self.pitch = getRoll(self.worldVertical, self.constructRight, self.constructForward)
    self.yaw = getRoll(self.worldVertical, self.constructUp, self.constructForward)
    self.heading = signedRotationAngle(self.worldVertical, self.constructForward, vec3(0,0,1)) % 360

    self.constructVelocity = vec3(self.construct.getWorldVelocity())
    self.constructVelocityDir = vec3(self.construct.getWorldVelocity()):normalize()
    --self.constructVerticalSpeed = constructVelocity:dot(constructUp)

    self.constructAngularVelocity = vec3(self.construct.getWorldAngularVelocity())
    --self.constructYawVelocity = constructAngularVelocity:dot(constructUp)

    self.currentAltitude = self.core.getAltitude()
end

function Avionics.updateEngineCommand(self)
    -- Engine commands
    local keepCollinearity = 1 -- for easier reading
    local dontKeepCollinearity = 0 -- for easier reading
    local tolerancePercentToSkipOtherPriorities = 1 -- if we are within this tolerance (in%), skip the next priorities

    -- Get current flight parameters
    self:getConstructFlightParameters()

    -- Rotation
    local angularAcceleration = self:computeCommandedAngularAcceleration()
    self.nav:setEngineTorqueCommand('torque', angularAcceleration, keepCollinearity, 'airfoil', '', '', tolerancePercentToSkipOtherPriorities)

    -- Longitudinal
    local longitudinalEngineTags = 'thrust analog longitudinal'
    local longitudinalAcceleration = self:computeCommandedLongitudinalAcceleration(longitudinalEngineTags)
    self.nav:setEngineForceCommand(longitudinalEngineTags, longitudinalAcceleration, keepCollinearity)

    -- Lateral
    local lateralEngineTags = 'thrust analog lateral'
    local lateralAcceleration = self:computeCommandedLateralAcceleration()
    self.nav:setEngineForceCommand(lateralEngineTags, lateralAcceleration, keepCollinearity)

    -- Vertical
    local verticalEngineTags = 'thrust analog vertical'
    if self.controlMode == avionicsMode.landing then
        verticalEngineTags = verticalEngineTags..', brake'
    end
    local verticalAcceleration = self:computeCommandedVerticalAcceleration()
    self.nav:setEngineForceCommand(verticalEngineTags, verticalAcceleration, keepCollinearity)

    -- brake
    local brakeTags = 'brake'
    local brakeAcceleration = self:computeCommandedBrakeAcceleration()
    self.nav:setEngineForceCommand(brakeTags, brakeAcceleration)
end

function Avionics.computeCommandedAngularAcceleration(self)
    local finalPitchInput = self.controlInputs.pitch + self.system.getControlDeviceForwardInput()
    local finalRollInput = self.controlInputs.roll + self.system.getControlDeviceYawInput()
    local finalYawInput = self.controlInputs.yaw - self.system.getControlDeviceLeftRightInput()

    local yawAxis = self.constructUp
    if self:getControlMode() == avionicsMode.ground then
        yawAxis = self.worldVertical
    end

    self.pid.pitch:inject(self.constructAngularVelocity:dot(self.constructRight))
    self.pid.roll:inject(self.constructAngularVelocity:dot(self.constructForward))
    self.pid.yaw:inject(self.constructAngularVelocity:dot(yawAxis))

    if self:getControlMode() == avionicsMode.ground then
        if self.controlFlags.pitch == avionicsInputFlag.active then
            local pitchCorrection = 0.2 * utils.clamp(normalizeAngle(self.maxPitchDeg * finalPitchInput - self.pitch), -10, 10)
            finalPitchInput = pitchCorrection - self.pid.pitch:get()
        elseif self.controlFlags.pitch == avionicsInputFlag.inactive then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
            if math.abs(self.pid.pitch:get()) < 0.01 then
                self.controlFlags.pitch = avionicsInputFlag.locked
            end
            finalPitchInput = finalPitchInput - self.pid.pitch:get()
        elseif self.controlFlags.pitch == avionicsInputFlag.locked then
            local pitchCorrection = 0.2 * utils.clamp(normalizeAngle(self.targetPitchDeg - self.pitch), -10, 10)
            finalPitchInput = finalPitchInput + pitchCorrection - self.pid.pitch:get()
        end

        if self.controlFlags.roll == avionicsInputFlag.active then
            local rollCorrection = 0.2 * utils.clamp(normalizeAngle(self.maxRollDeg * finalRollInput - self.roll), -10, 10)
            finalRollInput = rollCorrection - self.pid.roll:get()
        elseif self.controlFlags.roll == avionicsInputFlag.inactive then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
            if math.abs(self.pid.roll:get()) < 0.01 then
                self.controlFlags.roll = avionicsInputFlag.locked
            end
            finalRollInput = finalRollInput - self.pid.roll:get()
        elseif self.controlFlags.roll == avionicsInputFlag.locked then
            local rollCorrection = 0.2 * utils.clamp(normalizeAngle(self.targetRollDeg - self.roll), -10, 10)
            finalRollInput = finalRollInput + rollCorrection - self.pid.roll:get()
        end

        if self.controlFlags.yaw == avionicsInputFlag.inactive then
            --self.targetPitchDeg = self.pitch
            --self.targetRollDeg = self.roll
            --if math.abs(self.pid.yaw:get()) < 0.01 then
            --    self.controlFlags.yaw = avionicsInputFlag.locked
            --end
            finalYawInput = finalYawInput - self.pid.yaw:get()
        --elseif self.controlFlags.yaw == avionicsInputFlag.locked then
            --finalRollInput = finalRollInput - self.pid.yaw:get()
        end

    elseif self:getControlMode() == avionicsMode.atmosphere then
        local pitchCorrection = 0.2 * utils.clamp(normalizeAngle(self.targetPitchDeg - self.pitch), -10, 10)
        local rollCorrection = 0.2 * utils.clamp(normalizeAngle(self.targetRollDeg - self.roll), -10, 10)
        
        if self.controlFlags.pitch == avionicsInputFlag.inactive then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
            if math.abs(self.pid.pitch:get()) < 0.01 then
                self.controlFlags.pitch = avionicsInputFlag.locked
            end
            finalPitchInput = finalPitchInput - self.pid.pitch:get()
        elseif self.controlFlags.pitch == avionicsInputFlag.locked then
            finalPitchInput = finalPitchInput + pitchCorrection - self.pid.pitch:get()
        end

        if self.controlFlags.roll == avionicsInputFlag.inactive then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
            if math.abs(self.pid.roll:get()) < 0.01 then
                self.controlFlags.roll = avionicsInputFlag.locked
            end
            finalRollInput = finalRollInput - self.pid.roll:get()
        elseif self.controlFlags.roll == avionicsInputFlag.locked then
            finalRollInput = finalRollInput + rollCorrection - self.pid.roll:get()
        end
        
        if self.controlFlags.yaw == avionicsInputFlag.inactive then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
            if math.abs(self.pid.yaw:get()) < 0.01 then
                self.controlFlags.yaw = avionicsInputFlag.locked
            end
            finalYawInput = finalYawInput - self.pid.yaw:get()
        elseif self.controlFlags.yaw == avionicsInputFlag.locked then
            finalRollInput = finalRollInput - self.pid.yaw:get()
        end

    elseif self:getControlMode() == avionicsMode.space then
        if self.controlFlags.pitch == avionicsInputFlag.inactive then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
            finalPitchInput = finalPitchInput - self.pid.pitch:get()
        end
        if self.controlFlags.roll == avionicsInputFlag.inactive then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
            finalRollInput = finalRollInput - self.pid.roll:get()
        end

        if self.controlFlags.yaw == avionicsInputFlag.inactive then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
            finalYawInput = finalYawInput - self.pid.yaw:get()
        end

    elseif self:getControlMode() == avionicsMode.landing then
        if self.controlFlags.pitch ~= avionicsInputFlag.active then
            local pitchCorrection = 0.2 * utils.clamp(normalizeAngle(-self.pitch), -10, 10)
            finalPitchInput = pitchCorrection - self.pid.pitch:get()
        end
        if self.controlFlags.roll ~= avionicsInputFlag.active then
            local rollCorrection = 0.2 * utils.clamp(normalizeAngle(-self.roll), -10, 10)
            finalRollInput = rollCorrection - self.pid.roll:get()
        end
    end

    local targetAngularVelocity = finalPitchInput * self.controlAccelerationFactors.pitch * self.constructRight
                                    + finalRollInput * self.controlAccelerationFactors.roll * self.constructForward
                                    + finalYawInput * self.controlAccelerationFactors.yaw * yawAxis

    local angularAcceleration = targetAngularVelocity - self.constructAngularVelocity
    local airAcceleration = vec3(self.construct.getWorldAirFrictionAngularAcceleration())
    angularAcceleration = angularAcceleration - airAcceleration -- Try to compensate air friction
    return angularAcceleration
end

function Avionics.computeCommandedLongitudinalAcceleration(self, longitudinalEngineTags)
    local longitudinalAcceleration = vec3(0,0,0)
    local longitudinalCommandType = self.nav.axisCommandManager:getAxisCommandType(axisCommandId.longitudinal)
    if (longitudinalCommandType == axisCommandType.byThrottle) then
        longitudinalAcceleration = self.nav.axisCommandManager:composeAxisAccelerationFromThrottle(longitudinalEngineTags,axisCommandId.longitudinal)
    elseif  (longitudinalCommandType == axisCommandType.byTargetSpeed) then
        longitudinalAcceleration = self.nav.axisCommandManager:composeAxisAccelerationFromTargetSpeed(axisCommandId.longitudinal)
    end
    return longitudinalAcceleration
end

function Avionics.computeCommandedLateralAcceleration(self)
    local lateralVelocityThreshold = 1

    local finalLateralInput = self.controlInputs.lateral
    local lateralAcceleration = 0
    local lateralVelocity = self.constructVelocity:dot(self.constructRight)

    if (self.controlFlags.lateral == avionicsInputFlag.inactive) and (math.abs(lateralVelocity) > lateralVelocityThreshold) then
        self.pid.lateral:inject(-lateralVelocity)
        finalLateralInput = finalLateralInput + self.pid.lateral:get()
    end

    lateralAcceleration = (lateralAcceleration + finalLateralInput * self.controlAccelerationFactors.lateral * 9.81) * self.constructRight
    return lateralAcceleration
end

function Avionics.computeCommandedVerticalAcceleration(self)
    local finalVerticalInput = self.controlInputs.vertical
    local groundInput = self.controlInputs.ground
    local hoverDistance = self:getHoverDistance()
    local verticalVelocity = self.constructVelocity:dot(self.worldVertical)
    local verticalAcceleration = self.core.getGravityIntensity() * self.constructUp:dot(self.worldVertical)

    if self.controlFlags.ground == avionicsInputFlag.active then
        self.minGroundDistance = utils.clamp(hoverDistance or self.minGroundDistance, 0, self.maxGroundDistance)
        if self.nav.axisCommandManager.targetGroundAltitudeActivated == true then
            self.nav.axisCommandManager:setTargetGroundAltitude(self.minGroundDistance)
        end   
    end

    if self.controlFlags.vertical == avionicsInputFlag.inactive then
        local targetVerticalVelocity = 0
        local hoverAdjustment = 0
        if hoverDistance ~= nil then
            if self.nav.axisCommandManager.targetGroundAltitudeActivated == true then
                self.nav.axisCommandManager:deactivateGroundEngineAltitudeStabilization()
            end
            if self:getControlMode() == avionicsMode.landing then
                targetVerticalVelocity = -self.landingSpeed * utils.clamp(hoverDistance/self.landingDistance, 0, 1)
            else
                targetVerticalVelocity = self.constructVelocity:project_on(self.constructForward):dot(self.worldVertical) + groundInput * self.landingSpeed / 2
                self.pid.ground:inject(self.minGroundDistance - hoverDistance)
                hoverAdjustment = utils.clamp(self.pid.ground:get(), 0, 10)
            end
        else
            if self.nav.axisCommandManager.targetGroundAltitudeActivated == false then
                self.nav.axisCommandManager:activateGroundEngineAltitudeStabilization(self.minGroundDistance)
            end
            if self:getControlMode() == avionicsMode.landing then
                targetVerticalVelocity = -self.landingSpeed
            else
                targetVerticalVelocity = self.constructVelocity:project_on(self.constructForward):dot(self.worldVertical) + groundInput * self.landingSpeed / 2
            end
        end

        self.pid.vertical:inject(targetVerticalVelocity - verticalVelocity)
        finalVerticalInput = finalVerticalInput + hoverAdjustment + self.pid.vertical:get()
    end

    verticalAcceleration = (verticalAcceleration + finalVerticalInput * self.controlAccelerationFactors.vertical * 9.81) * self.constructUp
    return verticalAcceleration
end

function Avionics.computeCommandedBrakeAcceleration(self)
    local finalBrakeInput = self.controlInputs.brake
    if (finalBrakeInput == 0 and self.nav.axisCommandManager.throttle == 0 and self.constructVelocity:len() < self.autoBrakeSpeed) then
        finalBrakeInput = 1
    end
    if self.controlMode == avionicsMode.landing then
        local verticalVelocity = self.constructVelocity:dot(self.worldVertical)
        local horizontalVelocity = self.constructVelocity:project_on_plane(self.worldVertical):len()
        self.pid.brake:inject(utils.clamp(horizontalVelocity-0.1, 0, 10) + utils.clamp(-verticalVelocity - self.landingSpeed , 0, 10))
        finalBrakeInput = finalBrakeInput + self.pid.brake:get()
    end
    local brakeAcceleration = -finalBrakeInput *
                            (self.controlAccelerationFactors.brake.linear * self.constructVelocity
                            + self.controlAccelerationFactors.brake.flat * self.constructVelocityDir)
    return brakeAcceleration
end

function Avionics.controlInputStart(self, inputType, inputValue)
    inputName = reverseLookup(avionicsInputType, inputType)
    --self.controlInputs[inputName] = utils.clamp(self.controlInputs[inputName] + self.controlInputFactors[inputName].start * inputValue, -1, 1)
    self.controlInputs[inputName] = self.controlInputFactors[inputName].start * inputValue
    self:adjustControlParametersFromInputStart(inputType, inputName)
end

function Avionics.controlInputLoop(self, inputType, inputValue)
    inputName = reverseLookup(avionicsInputType, inputType)
    --self.controlInputs[inputName] = utils.clamp(self.controlInputs[inputName] + self.controlInputFactors[inputName].loop * inputValue, -1, 1)
    self.controlInputs[inputName] = self.controlInputFactors[inputName].loop * inputValue
    self:adjustControlParametersFromInputLoop(inputType, inputName)
end

function Avionics.controlInputStop(self, inputType)
    inputName = reverseLookup(avionicsInputType, inputType)
    self.controlInputs[inputName] = 0
    self:adjustControlParametersFromInputStop(inputType, inputName)
end

function Avionics.adjustControlParametersFromInputStart(self, inputType, inputName)
    local controlMode = self:getControlMode()
    self.controlFlags[inputName] = avionicsInputFlag.active
    if controlMode == avionicsMode.landing then
        if inputType == avionicsInputType.landing then
            local oldControlMode = self.controlMode
            self.controlMode = self.oldControlMode
            self.oldControlMode = oldControlMode
            self:autoLevel()
            self.nav.control.retractLandingGears()
            if self.nav.axisCommandManager.targetGroundAltitudeActivated == true then
                self.nav.axisCommandManager:setTargetGroundAltitude(self.minGroundDistance)
            end
        end
    elseif controlMode == avionicsMode.atmosphere then
        if inputType == avionicsInputType.ground then
            self.minGroundDistance = self:getHoverDistance() or self.minGroundDistance + self.controlInputs.ground
        end
        if inputType == avionicsInputType.pitch then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
        end
        if inputType == avionicsInputType.roll then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
        end
        if inputType == avionicsInputType.yaw then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
        end
        if inputType == avionicsInputType.landing then
            local oldControlMode = self.controlMode
            self.controlMode = avionicsMode.landing
            self.oldControlMode = oldControlMode
            self:autoLevel()
            self.nav.control.deployLandingGears()
            if self.nav.axisCommandManager.targetGroundAltitudeActivated == true then
                self.nav.axisCommandManager:setTargetGroundAltitude(0)
            end
        end
    elseif controlMode == avionicsMode.ground then
        if inputType == avionicsInputType.ground then
            self.minGroundDistance = self:getHoverDistance() or self.minGroundDistance + self.controlInputs.ground
        end
        if inputType == avionicsInputType.pitch then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
        end
        if inputType == avionicsInputType.roll then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
        end
        --if inputType == avionicsInputType.yaw then
            --self.targetPitchDeg = self.pitch
            --self.targetRollDeg = self.roll
        --end
        if inputType == avionicsInputType.landing then
            local oldControlMode = self.controlMode
            self.controlMode = avionicsMode.landing
            self.oldControlMode = oldControlMode
            self:autoLevel()
            self.nav.control.deployLandingGears()
            if self.nav.axisCommandManager.targetGroundAltitudeActivated == true then
                self.nav.axisCommandManager:setTargetGroundAltitude(0)
            end
        end
    end
end

function Avionics.adjustControlParametersFromInputLoop(self, inputType, inputName)
    local controlMode = self:getControlMode()
    self.controlFlags[inputName] = avionicsInputFlag.active
    if controlMode == avionicsMode.atmosphere then
        if inputType == avionicsInputType.pitch then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
        end
        if inputType == avionicsInputType.roll then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
        end
        if inputType == avionicsInputType.yaw then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
        end
        if inputType == avionicsInputType.ground then
            self.minGroundDistance = self:getHoverDistance() or self.minGroundDistance + self.controlInputs.ground / 10
        end
    elseif controlMode == avionicsMode.ground then                       
        if inputType == avionicsInputType.pitch then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
        end
        if inputType == avionicsInputType.roll then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
        end
        --if inputType == avionicsInputType.yaw then
            --self.targetPitchDeg = self.pitch
            --self.targetRollDeg = self.roll
        --end
        if inputType == avionicsInputType.ground then
            self.minGroundDistance = self:getHoverDistance() or self.minGroundDistance + self.controlInputs.ground / 10
        end
    end
end

function Avionics.adjustControlParametersFromInputStop(self, inputType, inputName)
    local controlMode = self:getControlMode()
    self.controlFlags[inputName] = avionicsInputFlag.inactive
    if controlMode == avionicsMode.atmosphere then                       
        if inputType == avionicsInputType.pitch then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
        end
        if inputType == avionicsInputType.roll then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
        end
        if inputType == avionicsInputType.yaw then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
        end
        if inputType == avionicsInputType.vertical then
            self.hoverAltitude = self.currentAltitude
        end
        if inputType == avionicsInputType.ground then
            self.minGroundDistance = self:getHoverDistance() or self.minGroundDistance
        end
    elseif controlMode == avionicsMode.ground then
        if inputType == avionicsInputType.pitch then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
        end
        if inputType == avionicsInputType.roll then
            self.targetPitchDeg = self.pitch
            self.targetRollDeg = self.roll
        end
        --if inputType == avionicsInputType.yaw then
            --self.targetPitchDeg = self.pitch
            --self.targetRollDeg = self.roll
        --end
        if inputType == avionicsInputType.vertical then
            self.hoverAltitude = self.currentAltitude
        end
        if inputType == avionicsInputType.ground then
            self.minGroundDistance = self:getHoverDistance() or self.minGroundDistance
        end
    end
end

function Avionics.autoLevel(self)
    self.controlFlags.pitch = avionicsInputFlag.locked
    self.controlFlags.roll = avionicsInputFlag.locked
    self.targetPitchDeg = 0
    self.targetRollDeg = 0
end

function Avionics.getHoverDistance(self)
    local hoverDistance = nil
    for k,v in pairs(self.hoverEngineLinks) do
        local d = v.getDistance()
        if (hoverDistance == nil) or (hoverDistance > d) then
            hoverDistance = d
        end
    end
    for k,v in pairs(self.databaseLinks) do
        if v.hasKey('groundDistance') == 1 then
            local distanceString = v.getStringValue('groundDistance')
            for k,d in pairs(json.decode(distanceString)) do
                if (hoverDistance == nil) or (hoverDistance > d) then
                    hoverDistance = d
                end
            end
        end
    end
    return hoverDistance
end