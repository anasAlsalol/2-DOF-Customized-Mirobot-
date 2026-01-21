-- ============================================
-- SCRIPT: child_script.lua
-- DESC: التحكم الديناميكي في ذراع 2-DOF مع دمج ROS 2
-- VERSION: 1.2 (مع التعديلات المطلوبة)
-- ============================================

sim = require 'sim'
simROS2 = require 'simROS2'

-- ============================================
-- PART 1: ANALYTICAL DYNAMIC MODELING
-- ============================================

-- معلمات DH للذراع 2-DOF المستوي
DH_params = {
    {alpha = 0,     a = 0.3,   d = 0,   theta_offset = 0},  -- Joint 1
    {alpha = 0,     a = 0.3,   d = 0,   theta_offset = 0}   -- Joint 2
}

-- معلمات الديناميكا
mass = {1.0, 1.0}           -- كتل الوصلات (kg)
link_length = {0.3, 0.3}    -- أطوال الوصلات (m)
g = 9.81                    -- تسارع الجاذبية
inertia = {                 -- لحظة القصور الذاتي (kg·m²)
    mass[1] * link_length[1]^2 / 12,
    mass[2] * link_length[2]^2 / 12
}

-- ============================================
-- متغيرات عالمية
-- ============================================
jointHandles = {}
tipHandle = -1
control_mode = "external_ros"  -- "internal", "external_ros", "hybrid"
ros_connected = false
ros_last_heartbeat = 0

-- لتسجيل البيانات للمقارنة
logData = {
    time = {},
    q1 = {}, q2 = {},
    q1_dot = {}, q2_dot = {},
    tau_lag1 = {}, tau_lag2 = {},
    tau_ne1 = {}, tau_ne2 = {},
    tau_sim1 = {}, tau_sim2 = {},
    error1 = {}, error2 = {},
    control_mode_log = {}
}

prev_q = {0, 0}
prev_time = 0
simulation_started = false

-- ROS integration variables
ros_joint_pub = nil
ros_tau_sub = nil
ros_heartbeat_sub = nil
ros_status_pub = nil
ros_trajectory_sub = nil
ros_control_mode_sub = nil
ros_handshake_sub = nil

external_torque = {0, 0}
external_trajectory = {
    positions = {0, 0},
    velocities = {0, 0},
    accelerations = {0, 0}
}

external_torque_received = false
trajectory_received = false
ros_handshake_received = false
ros_joint_names = {"joint1", "joint2"}

-- ============================================
-- PART 2: NEWTON-EULER VARIABLES
-- ============================================
w = {{0, 0, 0}, {0, 0, 0}}      -- السرعات الزاوية
wd = {{0, 0, 0}, {0, 0, 0}}     -- التسارعات الزاوية

ros_node = nil
ros_api_mode = "none" -- "node", "implicit", "legacy", "init", "none"

function sysCall_init()
    print("========================================")
    print("2-DOF MANIPULATOR DYNAMICS SIMULATION")
    print("========================================")
    
    -- Robust ROS2 initialization supporting multiple simROS2 API variants
    if simROS2 == nil then
        print("✗ simROS2 plugin not available")
        control_mode = "internal"
        ros_connected = false
    else
        local ok, err = pcall(function()
            -- prefer node-based API if available
            if type(simROS2.createNode) == "function" and type(simROS2.createPublisher) == "function" then
                ros_node = simROS2.createNode('/dynamics_sim_node')
                -- try node-based publisher/subscription creation
                local success_pubs = pcall(function()
                    -- Use only simple supported types: 'String' and 'Bool'
                    ros_joint_pub = simROS2.createPublisher(ros_node, '/robot/joint_states', 'String')
                    ros_status_pub = simROS2.createPublisher(ros_node, '/sim/status', 'String')
                    ros_tau_sub = simROS2.createSubscription(ros_node, '/robot/joint_torque_cmd', 'String', 'torqueCallback')
                    ros_heartbeat_sub = simROS2.createSubscription(ros_node, '/ros/heartbeat', 'Bool', 'heartbeatCallback')
                    ros_trajectory_sub = simROS2.createSubscription(ros_node, '/robot/reference_trajectory', 'String', 'trajectoryCallback')
                    ros_control_mode_sub = simROS2.createSubscription(ros_node, '/sim/control_mode', 'String', 'controlModeCallback')
                    ros_handshake_sub = simROS2.createSubscription(ros_node, '/dynamics/handshake', 'String', 'handshakeCallback')
                end)
                if success_pubs then
                    ros_api_mode = "node"
                else
                    -- fallback to non-node createPublisher/createSubscription variant
                    ros_joint_pub = simROS2.createPublisher('/robot/joint_states', 'String')
                    ros_status_pub = simROS2.createPublisher('/sim/status', 'String')
                    ros_tau_sub = simROS2.createSubscription('/robot/joint_torque_cmd', 'String', 'torqueCallback')
                    ros_heartbeat_sub = simROS2.createSubscription('/ros/heartbeat', 'Bool', 'heartbeatCallback')
                    ros_trajectory_sub = simROS2.createSubscription('/robot/reference_trajectory', 'String', 'trajectoryCallback')
                    ros_control_mode_sub = simROS2.createSubscription('/sim/control_mode', 'String', 'controlModeCallback')
                    ros_handshake_sub = simROS2.createSubscription('/dynamics/handshake', 'String', 'handshakeCallback')
                    ros_api_mode = "implicit"
                end

            elseif type(simROS2.createPublisher) == "function" and type(simROS2.createSubscription) == "function" then
                -- createPublisher/createSubscription without explicit node (simple types only)
                ros_joint_pub = simROS2.createPublisher('/robot/joint_states', 'String')
                ros_status_pub = simROS2.createPublisher('/sim/status', 'String')
                ros_tau_sub = simROS2.createSubscription('/robot/joint_torque_cmd', 'String', 'torqueCallback')
                ros_heartbeat_sub = simROS2.createSubscription('/ros/heartbeat', 'Bool', 'heartbeatCallback')
                ros_trajectory_sub = simROS2.createSubscription('/robot/reference_trajectory', 'String', 'trajectoryCallback')
                ros_control_mode_sub = simROS2.createSubscription('/sim/control_mode', 'String', 'controlModeCallback')
                ros_handshake_sub = simROS2.createSubscription('/dynamics/handshake', 'String', 'handshakeCallback')
                ros_api_mode = "implicit"

            elseif type(simROS2.advertise) == "function" and type(simROS2.subscribe) == "function" then
                -- legacy API (older plugin) - use simple types only
                ros_joint_pub = simROS2.advertise('/robot/joint_states', 'String')
                ros_status_pub = simROS2.advertise('/sim/status', 'String')
                ros_tau_sub = simROS2.subscribe('/robot/joint_torque_cmd', 'String', 'torqueCallback')
                ros_heartbeat_sub = simROS2.subscribe('/ros/heartbeat', 'Bool', 'heartbeatCallback')
                ros_trajectory_sub = simROS2.subscribe('/robot/reference_trajectory', 'String', 'trajectoryCallback')
                ros_control_mode_sub = simROS2.subscribe('/sim/control_mode', 'String', 'controlModeCallback')
                ros_handshake_sub = simROS2.subscribe('/dynamics/handshake', 'String', 'handshakeCallback')
                ros_api_mode = "legacy"

            else
                -- simROS2 does not expose an init/createPublisher/createNode/advertise API we recognize
                error("simROS2 present but no recognized API functions (createPublisher/createNode/advertise)")
            end
        end)

        if not ok then
            print("✗ ERROR: Failed to initialize simROS2 publishers/subscribers: " .. tostring(err))
            print("  Falling back to internal control mode")
            control_mode = "internal"
            ros_connected = false
            ros_api_mode = "none"
        else
            ros_connected = true
            ros_last_heartbeat = sim.getSimulationTime()
            print("✓ simROS2 topics initialized (mode: " .. ros_api_mode .. ")")
            print("  ROS_DOMAIN_ID: " .. (os.getenv("ROS_DOMAIN_ID") or "0"))
        end
    end

    -- 2. البحث الذكي عن مقابض المفاصل
    findRobotHandlesSmart()
    
    -- 3. تهيئة وضع التحكم
    setupControlMode()
    
    -- 4. عرض معلومات النظام
    printSystemInfo()
    
    -- 5. حساب وتحليل معلمات النموذج
    analyzeDynamicsModel()
    
    -- 6. Publish initial status
    publishStatus("Simulation initialized - Waiting for ROS connection...")
    
    simulation_started = true
end

function sysCall_sensing()
    -- Check ROS connection heartbeat
    local current_time = sim.getSimulationTime()
    if ros_connected and (current_time - ros_last_heartbeat) > 2.0 then
        print("⚠️ WARNING: ROS 2 heartbeat lost for " .. 
              string.format("%.1f", current_time - ros_last_heartbeat) .. " seconds")
        if control_mode == "external_ros" then
            control_mode = "internal"
            publishStatus("Switched to internal control - ROS connection lost")
        end
    end
end

function findRobotHandlesSmart()
    -- Direct-path-only joint lookup: try '/joint1' and '/joint2' explicitly
    print("Searching for joints via direct paths '/joint1' and '/joint2'")

    for i = 1, 2 do
        local success, handle = pcall(function()
            return sim.getObject('/joint' .. i)
        end)

        if success and handle ~= -1 then
            jointHandles[i] = handle
            print("✓ Found joint" .. i .. " via direct path")
        else
            print("⚠️ Direct path '/joint" .. i .. "' not found — creating dummy joint")
            jointHandles[i] = -i
        end
    end

    -- Search for tip as before
    searchForTip()
end

function searchForTip()
    -- البحث عن نهاية الذراع
    local baseHandle = sim.getObject('.')
    local allObjects = sim.getObjectsInTree(baseHandle, sim.handle_all, 1)
    
    local tipCandidates = {
        "tip", "Tip", "end", "End", "flange", "Flange",
        "link6", "link6_visible", "link7", "vacuumGripper"
    }
    
    for _, objHandle in ipairs(allObjects) do
        local objName = sim.getObjectAlias(objHandle, -1)
        
        for _, candidate in ipairs(tipCandidates) do
            if string.find(objName, candidate) then
                tipHandle = objHandle
                print("✓ Found tip: " .. objName)
                return
            end
        end
    end
    
    print("⚠️ Tip not found, using default")
end

function setupControlMode()
    print("\n=== SETTING CONTROL MODE ===")
    
    for i = 1, 2 do
        if jointHandles[i] and jointHandles[i] > 0 then
            -- التحقق إذا كان المقبض صالحًا
            local success = pcall(function()
                sim.setJointTargetForce(jointHandles[i], 100)
                sim.setJointTargetPosition(jointHandles[i], 9999)
                sim.setJointPosition(jointHandles[i], 0)
                return true
            end)
            
            if success then
                print("✓ Joint " .. i .. " set to TORQUE mode")
            else
                print("✗ Joint " .. i .. " control failed - using simulation mode")
                jointHandles[i] = -i
            end
        else
            print("⚠️ Joint " .. i .. " is simulated (no physical joint)")
        end
    end
    
    -- محاولة تثبيت المفاصل الأخرى
    for i = 3, 6 do
        local success, handle = pcall(function()
            return sim.getObject('/joint' .. i)
        end)
        
        if success and handle ~= -1 then
            pcall(function()
                sim.setJointTargetPosition(handle, 0)
                sim.setJointTargetForce(handle, 1000)
            end)
            print("✓ Joint " .. i .. " LOCKED")
        end
    end
end

function printSystemInfo()
    print("\n=== SYSTEM PARAMETERS ===")
    print("Project: Dynamics Modeling of 2-DOF Planar Arm")
    print("Selected Joints: First 2 joints of Mirobot")
    print("ROS 2 Integration: " .. (ros_connected and "ACTIVE" or "INACTIVE"))
    print("Control Mode: " .. control_mode)
    print("Dynamics Models: Lagrangian & Newton-Euler")
    print("Control: PD + Feedforward Dynamics Compensation")
    print("=========================\n")
end

function analyzeDynamicsModel()
    print("=== DYNAMICS MODEL ANALYSIS ===")
    
    -- معادلات Lagrangian
    print("\n1. LAGRANGIAN DYNAMICS:")
    print("   τ = M(q)q'' + C(q,q')q' + G(q)")
    
    -- معادلات Newton-Euler
    print("\n2. NEWTON-EULER ALGORITHM:")
    print("   Outward pass: Base → Tip (velocities)")
    print("   Inward pass: Tip → Base (forces)")
    
    -- معلمات النموذج
    print("\n3. MODEL PARAMETERS:")
    print("   Mass: [" .. mass[1] .. ", " .. mass[2] .. "] kg")
    print("   Length: [" .. link_length[1] .. ", " .. link_length[2] .. "] m")
    print("   Gravity: " .. g .. " m/s²")
    
    print("================================\n")
end

function sysCall_actuation()
    local simTime = sim.getSimulationTime()
    
    -- Wait for ROS handshake in external modes
    if (control_mode == "external_ros" or control_mode == "hybrid") and 
       not ros_handshake_received and simTime > 1.0 then
        if simTime % 1.0 < 0.05 then
            print("⏳ Waiting for ROS node handshake...")
        end
        -- Use internal control until handshake received
        control_mode = "internal"
    end
    
    -- تخطيط الحركة المرجعية
    local q_ref, qd_ref, qdd_ref
    if trajectory_received and (control_mode == "external_ros" or control_mode == "hybrid") then
        -- Use external trajectory from ROS
        q_ref = {external_trajectory.positions[1], external_trajectory.positions[2]}
        qd_ref = {external_trajectory.velocities[1], external_trajectory.velocities[2]}
        qdd_ref = {external_trajectory.accelerations[1], external_trajectory.accelerations[2]}
    else
        -- Generate internal trajectory
        q_ref, qd_ref, qdd_ref = generateReferenceTrajectory(simTime)
    end
    
    -- قراءة الوضعية الحالية
    local q_act, qd_act = readJointStates()
    
    -- حساب الأخطاء
    local error = {
        q_ref[1] - (q_act[1] or 0),
        q_ref[2] - (q_act[2] or 0)
    }
    
    local error_dot = {
        qd_ref[1] - (qd_act[1] or 0),
        qd_ref[2] - (qd_act[2] or 0)
    }
    
    -- حساب عزم Lagrangian و Newton-Euler للمقارنة
    local tau_lag = computeLagrangianTorque(q_ref, qd_ref, qdd_ref)
    local tau_ne = computeNewtonEulerTorque(q_ref, qd_ref, qdd_ref)
    
    -- PART 3: حساب عزم التحكم بناءً على وضع التحكم
    local tau_control
    if control_mode == "external_ros" then
        -- Use external torque from ROS
        if external_torque_received then
            tau_control = {external_torque[1] or 0, external_torque[2] or 0}
            -- Reset for next iteration
            external_torque_received = false
        else
            -- Fallback to internal control if no external torque received
            tau_control = computeControlTorque(q_act, qd_act, q_ref, qd_ref, qdd_ref, error, error_dot)
            if simTime > 2.0 and simTime % 2.0 < 0.05 then
                print("⚠️ Using internal control - no external torque received")
            end
        end
    elseif control_mode == "hybrid" then
        -- Combine internal and external control
        local tau_internal = computeControlTorque(q_act, qd_act, q_ref, qd_ref, qdd_ref, error, error_dot)
        if external_torque_received then
            -- Weighted combination (50% internal, 50% external)
            tau_control = {
                0.5 * tau_internal[1] + 0.5 * (external_torque[1] or 0),
                0.5 * tau_internal[2] + 0.5 * (external_torque[2] or 0)
            }
            external_torque_received = false
        else
            tau_control = tau_internal
        end
    else
        -- Internal control only
        tau_control = computeControlTorque(q_act, qd_act, q_ref, qd_ref, qdd_ref, error, error_dot)
    end
    
    -- تطبيق العزم
    applyJointTorques(tau_control)
    
    -- Publish JointState to ROS2
    if ros_connected and ros_joint_pub then
        -- Publish joint states as comma-separated String: "pos1,pos2,vel1,vel2,eff1,eff2"
        local s = string.format("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
            q_act[1] or 0, q_act[2] or 0,
            qd_act[1] or 0, qd_act[2] or 0,
            tau_control[1] or 0, tau_control[2] or 0
        )
        local js_msg = { data = s }
        simROS2.publish(ros_joint_pub, js_msg)
    end
    
    -- PART 5: تسجيل البيانات للمقارنة
    logSimulationData(simTime, q_act, qd_act, q_ref, tau_control, tau_lag, tau_ne, error)
    
    -- عرض المعلومات
    if simTime % 2.0 < 0.05 then
        displaySimulationInfo(simTime, q_act, q_ref, tau_lag, tau_ne, error)
    end
    
    -- حفظ البيانات كل 10 ثوان
    if math.floor(simTime) % 10 == 0 and math.floor(simTime) ~= math.floor(prev_time) then
        saveDataToFile()
    end
end

-- ============================================
-- PART 1: LAGRANGIAN DYNAMICS
-- ============================================

function computeInertiaMatrix(q)
    local c2 = math.cos(q[2])
    
    local M11 = mass[1] * link_length[1]^2 / 3 + 
                mass[2] * (link_length[1]^2 + link_length[2]^2 / 3 + 
                link_length[1] * link_length[2] * c2)
    
    local M12 = mass[2] * (link_length[2]^2 / 3 + 
                link_length[1] * link_length[2] * c2 / 2)
    
    local M22 = mass[2] * link_length[2]^2 / 3
    
    return {
        {M11, M12},
        {M12, M22}
    }
end

function computeCoriolisMatrix(q, qd)
    local s2 = math.sin(q[2])
    local h = -mass[2] * link_length[1] * link_length[2] * s2 / 2
    
    return {
        {h * qd[2], h * (qd[1] + qd[2])},
        {-h * qd[1], 0}
    }
end

function computeGravityVector(q)
    local c1 = math.cos(q[1])
    local c12 = math.cos(q[1] + q[2])
    
    local G1 = (mass[1] + mass[2]) * g * link_length[1] * c1 / 2 + 
               mass[2] * g * link_length[2] * c12 / 2
    
    local G2 = mass[2] * g * link_length[2] * c12 / 2
    
    return {G1, G2}
end

function computeLagrangianTorque(q, qd, qdd)
    local M = computeInertiaMatrix(q)
    local C = computeCoriolisMatrix(q, qd)
    local G = computeGravityVector(q)
    
    local M_qdd = {
        M[1][1] * qdd[1] + M[1][2] * qdd[2],
        M[2][1] * qdd[1] + M[2][2] * qdd[2]
    }
    
    local C_qd = {
        C[1][1] * qd[1] + C[1][2] * qd[2],
        C[2][1] * qd[1] + C[2][2] * qd[2]
    }
    
    return {
        M_qdd[1] + C_qd[1] + G[1],
        M_qdd[2] + C_qd[2] + G[2]
    }
end

-- ============================================
-- PART 2: NEWTON-EULER ALGORITHM
-- ============================================

function computeNewtonEulerTorque(q, qd, qdd)
    -- Outward Pass
    w[1] = {0, 0, qd[1]}
    wd[1] = {0, 0, qdd[1]}
    
    -- سرعة وتسارع مركز الكتلة للوصلة 1
    local r1 = {link_length[1]/2, 0, 0}
    local vd1 = crossProduct(wd[1], r1)
    vd1 = vectorAdd(vd1, crossProduct(w[1], crossProduct(w[1], r1)))
    vd1 = vectorAdd(vd1, {0, -g, 0})
    
    -- Outward للمفصل 2
    local R = rotationMatrixZ(q[2])
    w[2] = vectorAdd(rotateVector(w[1], R), {0, 0, qd[2]})
    
    local term1 = rotateVector(wd[1], R)
    local term2 = {0, 0, qdd[2]}
    local term3 = crossProduct(rotateVector(w[1], R), {0, 0, qd[2]})
    wd[2] = vectorAdd(vectorAdd(term1, term2), term3)
    
    -- Inward Pass
    local r2 = {link_length[2]/2, 0, 0}
    local vd2_local = crossProduct(wd[2], r2)
    vd2_local = vectorAdd(vd2_local, crossProduct(w[2], crossProduct(w[2], r2)))
    
    local vd2 = rotateVector(vd2_local, rotationMatrixZ(q[1] + q[2]))
    vd2 = vectorAdd(vd2, {0, -g, 0})
    
    -- القوى والعزوم
    local f2 = {mass[2] * vd2[1], mass[2] * vd2[2], 0}
    local n2 = {0, 0, inertia[2] * wd[2][3]}
    
    local tau2 = n2[3] + (link_length[2]/2) * f2[2]
    
    local f2_in_frame1 = rotateVector(f2, rotationMatrixZ(-q[2]))
    local f1_linear = {mass[1] * vd1[1], mass[1] * vd1[2], 0}
    local f1 = vectorAdd(f1_linear, f2_in_frame1)
    
    local n1 = {0, 0, inertia[1] * wd[1][3]}
    local r1_to_2 = {link_length[1], 0, 0}
    local moment_f2 = crossProduct(r1_to_2, f2_in_frame1)
    
    local total_moment = vectorAdd(n1, moment_f2)
    total_moment = vectorAdd(total_moment, n2)
    
    local tau1 = total_moment[3] + (link_length[1]/2) * f1_linear[2]
    
    return {tau1, tau2}
end

-- ============================================
-- PART 3: TRAJECTORY AND CONTROL
-- ============================================

function generateReferenceTrajectory(t)
    local q_ref = {0, 0}
    local qd_ref = {0, 0}
    local qdd_ref = {0, 0}
    
    local motionType = "sine"  -- Can be changed dynamically
    
    if motionType == "sine" then
        local A1, A2 = 0.5, 0.3
        local f1, f2 = 0.5, 0.8
        
        q_ref[1] = A1 * math.sin(2 * math.pi * f1 * t)
        q_ref[2] = A2 * math.sin(2 * math.pi * f2 * t + math.pi/4)
        
        qd_ref[1] = A1 * 2 * math.pi * f1 * math.cos(2 * math.pi * f1 * t)
        qd_ref[2] = A2 * 2 * math.pi * f2 * math.cos(2 * math.pi * f2 * t + math.pi/4)
        
        qdd_ref[1] = -A1 * (2 * math.pi * f1)^2 * math.sin(2 * math.pi * f1 * t)
        qdd_ref[2] = -A2 * (2 * math.pi * f2)^2 * math.sin(2 * math.pi * f2 * t + math.pi/4)
        
    elseif motionType == "circle" then
        local radius = 0.1
        local frequency = 0.3
        local center_x = 0.45
        
        local x = center_x + radius * math.cos(2 * math.pi * frequency * t)
        local y = radius * math.sin(2 * math.pi * frequency * t)
        
        local L1, L2 = link_length[1], link_length[2]
        local D = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2)
        D = math.max(-0.9999, math.min(0.9999, D))
        
        q_ref[2] = math.acos(D)
        q_ref[1] = math.atan2(y, x) - math.atan2(L2 * math.sin(q_ref[2]), L1 + L2 * math.cos(q_ref[2]))
        
    else -- point-to-point
        local t_total = 4.0
        local t_norm = (t % t_total) / t_total
        
        if t_norm < 0.25 then
            local s = t_norm / 0.25
            q_ref[1] = cubicInterpolation(0, 0.8, 0, 0, s)
            q_ref[2] = cubicInterpolation(0, 0.4, 0, 0, s)
        elseif t_norm < 0.5 then
            local s = (t_norm - 0.25) / 0.25
            q_ref[1] = cubicInterpolation(0.8, 0.4, 0, 0, s)
            q_ref[2] = cubicInterpolation(0.4, 0.6, 0, 0, s)
        elseif t_norm < 0.75 then
            local s = (t_norm - 0.5) / 0.25
            q_ref[1] = cubicInterpolation(0.4, -0.3, 0, 0, s)
            q_ref[2] = cubicInterpolation(0.6, 0.2, 0, 0, s)
        else
            local s = (t_norm - 0.75) / 0.25
            q_ref[1] = cubicInterpolation(-0.3, 0, 0, 0, s)
            q_ref[2] = cubicInterpolation(0.2, 0, 0, 0, s)
        end
    end
    
    return q_ref, qd_ref, qdd_ref
end

function computeControlTorque(q_act, qd_act, q_ref, qd_ref, qdd_ref, error, error_dot)
    local Kp = {150.0, 120.0}
    local Kd = {25.0, 20.0}
    
    local tau_pd = {
        Kp[1] * error[1] + Kd[1] * error_dot[1],
        Kp[2] * error[2] + Kd[2] * error_dot[2]
    }
    
    local tau_ff = computeLagrangianTorque(q_ref, qd_ref, qdd_ref)
    
    local tau_total = {
        tau_pd[1] + tau_ff[1],
        tau_pd[2] + tau_ff[2]
    }
    
    -- تقييد العزم
    local max_torque = 50.0
    tau_total[1] = math.max(-max_torque, math.min(max_torque, tau_total[1]))
    tau_total[2] = math.max(-max_torque, math.min(max_torque, tau_total[2]))
    
    return tau_total
end

function readJointStates()
    local q = {0, 0}
    local qd = {0, 0}
    
    for i = 1, 2 do
        if jointHandles[i] and jointHandles[i] > 0 then
            local success, position = pcall(function()
                return sim.getJointPosition(jointHandles[i])
            end)
            
            if success then
                q[i] = position
                
                local current_time = sim.getSimulationTime()
                local dt = current_time - prev_time
                
                if dt > 0.001 and prev_time > 0 then
                    qd[i] = (q[i] - (prev_q[i] or 0)) / dt
                end
            end
        else
            -- قيم محاكاة إذا لم يكن هناك مفصل حقيقي
            q[i] = 0.5 * math.sin(sim.getSimulationTime() * 0.5 + i * 0.5)
            qd[i] = 0.25 * math.cos(sim.getSimulationTime() * 0.5 + i * 0.5)
        end
    end
    
    prev_q = {q[1], q[2]}
    prev_time = sim.getSimulationTime()
    
    return q, qd
end

function applyJointTorques(tau)
    for i = 1, 2 do
        if jointHandles[i] and jointHandles[i] > 0 then
            pcall(function()
                sim.setJointForce(jointHandles[i], tau[i])
            end)
        end
    end
end

-- ============================================
-- PART 5: DATA LOGGING AND ANALYSIS
-- ============================================

function logSimulationData(time, q_act, qd_act, q_ref, tau_control, tau_lag, tau_ne, error)
    table.insert(logData.time, time)
    table.insert(logData.q1, q_act[1] or 0)
    table.insert(logData.q2, q_act[2] or 0)
    table.insert(logData.q1_dot, qd_act[1] or 0)
    table.insert(logData.q2_dot, qd_act[2] or 0)
    table.insert(logData.tau_lag1, tau_lag[1] or 0)
    table.insert(logData.tau_lag2, tau_lag[2] or 0)
    table.insert(logData.tau_ne1, tau_ne[1] or 0)
    table.insert(logData.tau_ne2, tau_ne[2] or 0)
    table.insert(logData.tau_sim1, tau_control[1] or 0)
    table.insert(logData.tau_sim2, tau_control[2] or 0)
    table.insert(logData.error1, error[1] or 0)
    table.insert(logData.error2, error[2] or 0)
    table.insert(logData.control_mode_log, control_mode)
end

function displaySimulationInfo(time, q_act, q_ref, tau_lag, tau_ne, error)
    print(string.format("\n[%.1f s] Control: %s | ROS: %s", 
          time, control_mode, ros_connected and "Connected" or "Disconnected"))
    print(string.format("Pos: [%.3f, %.3f] | Ref: [%.3f, %.3f]", 
          q_act[1] or 0, q_act[2] or 0, q_ref[1] or 0, q_ref[2] or 0))
    print(string.format("Error: [%.4f, %.4f] rad", error[1] or 0, error[2] or 0))
    print(string.format("Lagrangian τ: [%.2f, %.2f] N·m", tau_lag[1] or 0, tau_lag[2] or 0))
    print(string.format("Newton-Euler τ: [%.2f, %.2f] N·m", tau_ne[1] or 0, tau_ne[2] or 0))
    
    local diff1 = math.abs((tau_lag[1] or 0) - (tau_ne[1] or 0))
    local diff2 = math.abs((tau_lag[2] or 0) - (tau_ne[2] or 0))
    print(string.format("Difference: [%.4f, %.4f] N·m", diff1, diff2))
end

function saveDataToFile()
    if #logData.time == 0 then return end
    
    local scenePath = sim.getStringParam(sim.stringparam_scene_path)
    local filename = scenePath .. "/dynamics_comparison_ros.csv"
    
    local file, err = io.open(filename, "w")
    if file then
        file:write("time,control_mode,q1,q2,q1_dot,q2_dot,tau_lag1,tau_lag2,tau_ne1,tau_ne2,tau_sim1,tau_sim2,error1,error2\n")
        
        for i = 1, #logData.time do
            file:write(string.format("%.4f,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                logData.time[i],
                logData.control_mode_log[i] or "unknown",
                logData.q1[i] or 0,
                logData.q2[i] or 0,
                logData.q1_dot[i] or 0,
                logData.q2_dot[i] or 0,
                logData.tau_lag1[i] or 0,
                logData.tau_lag2[i] or 0,
                logData.tau_ne1[i] or 0,
                logData.tau_ne2[i] or 0,
                logData.tau_sim1[i] or 0,
                logData.tau_sim2[i] or 0,
                logData.error1[i] or 0,
                logData.error2[i] or 0
            ))
        end
        
        file:close()
        print("\n✓ Data saved: " .. filename .. " (" .. #logData.time .. " samples)")
    else
        print("✗ Could not save data: " .. (err or "unknown error"))
    end
end

-- ============================================
-- ROS 2 CALLBACK FUNCTIONS
-- ============================================

function torqueCallback(msg)
    -- msg expected either as String CSV "tau1,tau2" or table { data = {tau1, tau2} }
    if msg and msg.data then
        local d = msg.data
        if type(d) == 'string' then
            d = parseNumberList(d)
        end
        external_torque[1] = d[1] or 0
        external_torque[2] = d[2] or 0
        external_torque_received = true
        -- Reset heartbeat since we received a message
        ros_last_heartbeat = sim.getSimulationTime()
    end
end

function heartbeatCallback(msg)
    if msg and msg.data ~= nil then
        ros_last_heartbeat = sim.getSimulationTime()
        if not ros_connected then
            ros_connected = true
            publishStatus("ROS 2 reconnected")
        end
    end
end

function trajectoryCallback(msg)
    -- Expect String CSV: "pos1,pos2,vel1,vel2,acc1,acc2" or table-like data
    if msg and msg.data then
        local d = msg.data
        if type(d) == 'string' then
            d = parseNumberList(d)
        end
        if #d >= 2 then
            external_trajectory.positions[1] = d[1]
            external_trajectory.positions[2] = d[2]
        end
        if #d >= 4 then
            external_trajectory.velocities[1] = d[3]
            external_trajectory.velocities[2] = d[4]
        end
        if #d >= 6 then
            external_trajectory.accelerations[1] = d[5]
            external_trajectory.accelerations[2] = d[6]
        end

        trajectory_received = true
        ros_last_heartbeat = sim.getSimulationTime()
    end
end

function controlModeCallback(msg)
    if msg and msg.data then
        local new_mode = msg.data
        if new_mode == "internal" or new_mode == "external_ros" or new_mode == "hybrid" then
            control_mode = new_mode
            publishStatus("Control mode changed to: " .. control_mode)
            print("✓ Control mode changed to: " .. control_mode)
        else
            print("⚠️ WARNING: Invalid control mode: " .. new_mode)
        end
    end
end

function handshakeCallback(msg)
    if msg and msg.data then
        if msg.data == "READY" then
            ros_handshake_received = true
            print("✓ ROS node handshake received")
            publishStatus("ROS node connected - Starting external control")
            
            -- Switch back to external mode if we were waiting
            if control_mode == "internal" then
                control_mode = "external_ros"
                print("✓ Switching to external_ros control mode")
            end
        end
    end
end

function publishStatus(message)
    if ros_connected and ros_status_pub then
        local status_msg = {
            data = string.format("[%.2f s] %s", sim.getSimulationTime(), message)
        }
        simROS2.publish(ros_status_pub, status_msg)
    end
end

-- ============================================
-- HELPER FUNCTIONS
-- ============================================

-- Parse a comma-separated list of numbers in a String message
function parseNumberList(s)
    local out = {}
    if not s then return out end
    for token in string.gmatch(s, "([^,]+)") do
        local v = tonumber(token)
        table.insert(out, v or 0)
    end
    return out
end


function vectorAdd(a, b)
    return {a[1] + b[1], a[2] + b[2], a[3] + b[3]}
end

function crossProduct(a, b)
    return {
        a[2]*b[3] - a[3]*b[2],
        a[3]*b[1] - a[1]*b[3],
        a[1]*b[2] - a[2]*b[1]
    }
end

function rotateVector(v, R)
    return {
        R[1][1]*v[1] + R[1][2]*v[2] + R[1][3]*v[3],
        R[2][1]*v[1] + R[2][2]*v[2] + R[2][3]*v[3],
        R[3][1]*v[1] + R[3][2]*v[2] + R[3][3]*v[3]
    }
end

function rotationMatrixZ(theta)
    local c = math.cos(theta)
    local s = math.sin(theta)
    return {
        {c, -s, 0},
        {s, c, 0},
        {0, 0, 1}
    }
end

function cubicInterpolation(p0, p1, v0, v1, t)
    local h00 = 2*t^3 - 3*t^2 + 1
    local h10 = t^3 - 2*t^2 + t
    local h01 = -2*t^3 + 3*t^2
    local h11 = t^3 - t^2
    return h00*p0 + h10*v0 + h01*p1 + h11*v1
end

function sysCall_cleanup()
    print("\n========================================")
    print("SIMULATION TERMINATED")
    print("========================================")
    
    saveDataToFile()

    -- Clean up simROS2 resources depending on detected API
    if simROS2 then
        pcall(function()
            if ros_api_mode == "node" and ros_node and type(simROS2.destroyNode) == "function" then
                simROS2.destroyNode(ros_node)
            elseif type(simROS2.shutdown) == "function" then
                simROS2.shutdown()
            end
        end)
    end

    if #logData.time > 0 then
        local total_time = logData.time[#logData.time]
        local avg_error1 = 0
        local avg_error2 = 0
        
        for i = 1, #logData.time do
            avg_error1 = avg_error1 + math.abs(logData.error1[i] or 0)
            avg_error2 = avg_error2 + math.abs(logData.error2[i] or 0)
        end
        
        avg_error1 = avg_error1 / #logData.time
        avg_error2 = avg_error2 / #logData.time
        
        print("Final Statistics:")
        print("- Total simulation time: " .. string.format("%.2f", total_time) .. " s")
        print("- Data points recorded: " .. #logData.time)
        print("- ROS 2 Connected: " .. (ros_connected and "YES" or "NO"))
        print("- Final Control Mode: " .. control_mode)
        print("- Average tracking error:")
        print("  Joint 1: " .. string.format("%.6f", avg_error1) .. " rad")
        print("  Joint 2: " .. string.format("%.6f", avg_error2) .. " rad")
        print("\n✓ Data saved for analysis in Part 5")
    end
    
    print("========================================")
end