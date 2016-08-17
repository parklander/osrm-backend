-- Testbot, with turn penalty
-- Used for testing turn penalties

require 'testbot'

properties.left_hand_driving = true

local turn_penalty           = 50
local turn_bias              = properties.left_hand_driving and 1/1.2 or 1.2

function turn_function (turn)
  ---- compute turn penalty as angle^2, with a left/right bias
  k = turn.angle * turn.angle * turn_penalty / (90.0 * 90.0)
  if turn.angle >= 0 then
    turn.weight = k / turn_bias
  else
    turn.weight = k * turn_bias
  end
end
