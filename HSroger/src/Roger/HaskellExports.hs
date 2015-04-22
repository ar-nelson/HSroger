{-# LANGUAGE CPP                      #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE UnicodeSyntax            #-}

module Roger.HaskellExports where

import           Data.Vec
import           Foreign.C.Types
import           Foreign.Ptr
import           Foreign.StablePtr
import           Foreign.Storable

import qualified Roger.Project1    as Project1
import qualified Roger.Project2    as Project2
import qualified Roger.Project3    as Project3
import qualified Roger.Project4    as Project4
import qualified Roger.Project5    as Project5
import qualified Roger.Project6    as Project6
import qualified Roger.Project7    as Project7
import qualified Roger.Project8    as Project8
import qualified Roger.Project9    as Project9
import           Roger.Robot
import           Roger.Types

--------------------------------------------------------------------------------

type ControlFn s = Ptr Robot → StablePtr s → CDouble → IO (StablePtr s)
type ParamsFn  s = StablePtr s → IO (StablePtr s)
type ResetFn   s = Ptr Robot → StablePtr s → IO (StablePtr s)

controlFn ∷ (Robot → a → Double → IO (Robot, a)) → ControlFn a
controlFn fn rogerPtr statePtr (CDouble time) =
  do roger ← peek rogerPtr
     st    ← deRefStablePtr statePtr
     freeStablePtr statePtr
     (roger', st') ← fn roger st time
     poke rogerPtr roger'
     newStablePtr st'

paramsFn ∷ (a → IO a) → ParamsFn a
paramsFn fn ptr =
  do st  ← deRefStablePtr ptr
     freeStablePtr ptr
     st' ← fn st
     newStablePtr st'

--------------------------------------------------------------------------------

foreign export ccall hs_control_roger ∷ Ptr Robot → CDouble → IO ()

foreign export ccall hs_project2_control      ∷ ControlFn Project2.State
foreign export ccall hs_project2_enter_params ∷ ParamsFn Project2.State
foreign export ccall hs_project2_init_state   ∷ IO (StablePtr Project2.State)
foreign export ccall hs_inv_arm_kinematics    ∷ Ptr Robot
                                              → StablePtr Project2.State
                                              → CInt
                                              → CDouble
                                              → CDouble
                                              → Ptr CInt
                                              → IO (StablePtr Project2.State)

foreign export ccall hs_project3_control      ∷ ControlFn Project3.State
foreign export ccall hs_project3_enter_params ∷ ParamsFn Project3.State
foreign export ccall hs_project3_init_state   ∷ IO (StablePtr Project3.State)

foreign export ccall hs_project4_control      ∷ ControlFn Project4.State
foreign export ccall hs_project4_enter_params ∷ ParamsFn Project4.State
foreign export ccall hs_project4_init_state   ∷ IO (StablePtr Project4.State)

foreign export ccall hs_project5_control         ∷ ControlFn Project5.State
foreign export ccall hs_project5_enter_params    ∷ ParamsFn Project5.State
foreign export ccall hs_project5_init_state      ∷ IO (StablePtr Project5.State)
foreign export ccall hs_project5_get_observation ∷ StablePtr Project5.State
                                                 → Ptr Observation → IO ()

foreign export ccall hs_project6_control      ∷ ControlFn Project6.State
foreign export ccall hs_project6_enter_params ∷ ParamsFn Project6.State
foreign export ccall hs_project6_init_state   ∷ IO (StablePtr Project6.State)

foreign export ccall hs_project7_control      ∷ ControlFn Project7.State
foreign export ccall hs_project7_enter_params ∷ ParamsFn Project7.State
foreign export ccall hs_project7_init_state   ∷ IO (StablePtr Project7.State)

foreign export ccall hs_project8_control      ∷ ControlFn Project8.State
foreign export ccall hs_project8_enter_params ∷ ParamsFn Project8.State
foreign export ccall hs_project8_init_state   ∷ IO (StablePtr Project8.State)

foreign export ccall hs_project9_control      ∷ ControlFn Project9.State
foreign export ccall hs_project9_enter_params ∷ ParamsFn Project9.State
foreign export ccall hs_project9_init_state   ∷ IO (StablePtr Project9.State)

--------------------------------------------------------------------------------

hs_control_roger ∷ Ptr Robot → CDouble → IO ()
hs_control_roger rogerPtr (CDouble time) =
  do roger  ← peek rogerPtr
     roger' ← Project1.control roger time
     poke rogerPtr roger'

hs_project2_control      ∷ ControlFn Project2.State
hs_project2_enter_params ∷ ParamsFn Project2.State
hs_project2_init_state   ∷ IO (StablePtr Project2.State)
hs_inv_arm_kinematics    ∷ Ptr Robot
                         → StablePtr Project2.State
                         → CInt
                         → CDouble
                         → CDouble
                         → Ptr CInt
                         → IO (StablePtr Project2.State)
hs_project2_control      = controlFn Project2.control
hs_project2_enter_params = paramsFn Project2.enterParams
hs_project2_init_state   = Project2.initState >>= newStablePtr
hs_inv_arm_kinematics rogerPtr st limbN (CDouble x) (CDouble y) out =
  do roger  ← peek rogerPtr
     case Project2.invArmKinematics roger limb vec of
       Just setPt → poke rogerPtr roger { armSetpoint =
                         if limbN == 0
                            then (armSetpoint roger) { left  = setPt }
                            else (armSetpoint roger) { right = setPt }
                       } >> poke out 1
       Nothing → poke out 0
     return st
  where limb = if limbN == 0 then left else right
        vec  = Vec2D x y

hs_project3_control      ∷ ControlFn Project3.State
hs_project3_enter_params ∷ ParamsFn Project3.State
hs_project3_init_state   ∷ IO (StablePtr Project3.State)
hs_project3_control      = controlFn Project3.control
hs_project3_enter_params = paramsFn Project3.enterParams
hs_project3_init_state   = Project3.initState >>= newStablePtr

hs_project4_control      ∷ ControlFn Project4.State
hs_project4_enter_params ∷ ParamsFn Project4.State
hs_project4_init_state   ∷ IO (StablePtr Project4.State)
hs_project4_control      = controlFn Project4.control
hs_project4_enter_params = paramsFn Project4.enterParams
hs_project4_init_state   = Project4.initState >>= newStablePtr

hs_project5_control         ∷ ControlFn Project5.State
hs_project5_enter_params    ∷ ParamsFn Project5.State
hs_project5_init_state      ∷ IO (StablePtr Project5.State)
hs_project5_get_observation ∷ StablePtr Project5.State → Ptr Observation → IO ()
hs_project5_control         = controlFn Project5.control
hs_project5_enter_params    = paramsFn Project5.enterParams
hs_project5_init_state      = Project5.initState >>= newStablePtr
hs_project5_get_observation stPtr obsPtr =
  do st ← deRefStablePtr stPtr
     poke obsPtr (Project5.getObservation st)

hs_project6_control      ∷ ControlFn Project6.State
hs_project6_enter_params ∷ ParamsFn Project6.State
hs_project6_init_state   ∷ IO (StablePtr Project6.State)
hs_project6_control      = controlFn Project6.control
hs_project6_enter_params = paramsFn Project6.enterParams
hs_project6_init_state   = Project6.initState >>= newStablePtr

hs_project7_control      ∷ ControlFn Project7.State
hs_project7_enter_params ∷ ParamsFn Project7.State
hs_project7_init_state   ∷ IO (StablePtr Project7.State)
hs_project7_control      = controlFn Project7.control
hs_project7_enter_params = paramsFn Project7.enterParams
hs_project7_init_state   = Project7.initState >>= newStablePtr

hs_project8_control      ∷ ControlFn Project8.State
hs_project8_enter_params ∷ ParamsFn Project8.State
hs_project8_init_state   ∷ IO (StablePtr Project8.State)
hs_project8_control      = controlFn Project8.control
hs_project8_enter_params = paramsFn Project8.enterParams
hs_project8_init_state   = Project8.initState >>= newStablePtr

hs_project9_control      ∷ ControlFn Project9.State
hs_project9_enter_params ∷ ParamsFn Project9.State
hs_project9_init_state   ∷ IO (StablePtr Project9.State)
hs_project9_control      = controlFn Project9.control
hs_project9_enter_params = paramsFn Project9.enterParams
hs_project9_init_state   = Project9.initState >>= newStablePtr
