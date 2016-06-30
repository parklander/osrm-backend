-- Testbot, with turn penalty
-- Used for testing turn penalties

require 'testbot'

function turn_function (turn)
    turn.weight = 200 * math.abs(turn.angle) / 180 -- penalty 
end
