-- Быстрое вращение персонажа без вращения камеры (KRNL или LocalScript)

local plr = game.Players.LocalPlayer
local cam = workspace.CurrentCamera
local char = plr.Character or plr.CharacterAdded:Wait()
local hrp = char:WaitForChild("HumanoidRootPart")

local RunService = game:GetService("RunService")
local speed = math.rad(2000)

-- Сохраняем фиксированное направление камеры
local fixedCamCFrame = cam.CFrame

-- Блокируем камеру
cam.CameraType = Enum.CameraType.Scriptable
cam.CFrame = fixedCamCFrame

RunService.RenderStepped:Connect(function(dt)
	if hrp then
		-- Вращаем персонажа
		hrp.CFrame = hrp.CFrame * CFrame.Angles(0, speed * dt, 0)

		-- Удерживаем камеру на месте
		cam.CFrame = fixedCamCFrame
	end
end)
