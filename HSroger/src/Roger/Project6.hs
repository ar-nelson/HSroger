module Roger.Project6(State, control, enterParams, initState) where

import           Roger.Robot

type State = ()

control :: Robot -> State -> Double -> IO (Robot, State)
control r s _ = return (r, s)

enterParams :: State -> IO State
enterParams = return

initState :: IO State
initState = return ()

