{-# LANGUAGE FlexibleContexts      #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE RecordWildCards       #-}
{-# LANGUAGE UnicodeSyntax         #-}

module Roger.Project9(State, control, enterParams, initState) where

import           Control.Applicative
import           Control.Arrow        hiding (left, right)
import           Control.Category
import           Control.Monad.Reader
import           Control.Monad.State  hiding (State)
import           Data.Maybe           (fromMaybe)
import           Data.Vec             hiding (get)
import           Prelude              hiding (id, (.))
import           Roger.Robot
import           Roger.Types
import           Roger.Wire

type State = Wire (ReaderT Time IO) Robot Robot

initState ∷ IO State
initState = return pong

--------------------------------------------------------------------------------

setup ∷ (MonadIO m) ⇒ Wire m Robot Robot
setup = stateWire (skip (wire (const get) >>> ifWire id)
               >>> action (liftIO (putStrLn "Doing initial setup..."))
               >>> arr (\r → r { baseSetpoint = basePosition r
                               , armSetpoint  = armθ r
                               })
               >>> action (put False)) True

pong ∷ State
pong = setup <|> id

--------------------------------------------------------------------------------

control :: Robot -> State -> Double -> IO (Robot, State)
control roger st t = do (roger', st') ← runReaderT (runWire st roger) (Time t)
                        return (fromMaybe roger roger', st')

enterParams :: State -> IO State
enterParams = return

