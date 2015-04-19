module Roger.Project4(State, control, enterParams, reset, initState) where

import           Roger.Robot

type State = ()

control :: Robot -> State -> Double -> IO (Robot, State)
control r s _ = return (r, s)

enterParams :: State -> IO State
enterParams = return

reset :: Robot -> State -> IO (Robot, State)
reset r s = return (r, s)

initState :: IO State
initState = return ()

