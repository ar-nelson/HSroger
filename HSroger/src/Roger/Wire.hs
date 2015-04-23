{-# LANGUAGE FlexibleContexts      #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE TupleSections         #-}
{-# LANGUAGE UnicodeSyntax         #-}

module Roger.Wire(module Roger.Wire) where

import           Control.Applicative
import           Control.Arrow
import           Control.Category
import           Control.Monad
import           Control.Monad.Reader
import           Control.Monad.State
import           Prelude              hiding (id, (.))

--------------------------------------------------------------------------------
-- WIRE PATTERN
--
-- A super-simple implementation of the Wire pattern for functional reactive
-- programming. Like the NetWire library, but much smaller.
--
-- http://stackoverflow.com/a/13919666/548027 explains how this works.

newtype Wire m a b = Wire { runWire ∷ a → m (Maybe b, Wire m a b) }

evalWire ∷ (Monad m) ⇒ Wire m a b → a → m (Maybe b)
evalWire w = liftM fst . runWire w

execWire ∷ (Monad m) ⇒ Wire m a b → a → m (Wire m a b)
execWire w = liftM snd . runWire w

--------------------------------------------------------------------------------

instance (Monad m) ⇒ Functor (Wire m a) where
  fmap fn (Wire wfn) = Wire $ \a →
    do (b, w) ← wfn a
       return (fmap fn b, fmap fn w)

instance (Monad m) ⇒ Applicative (Wire m a) where
  pure v = w where w = Wire (const $ return (Just v, w))
  Wire ffn <*> Wire fv = Wire $ \a →
    do (fn, fnw) ← ffn a
       (v, vw)   ← fv a
       return (fn <*> v, fnw <*> vw)

instance (Monad m) ⇒ Category (Wire m) where
  id = w where w = Wire (\a → return (Just a, w))
  Wire f . Wire g = Wire $ \a →
    do (mb, g') ← g a
       case mb of
         Just b  → do (c, f') ← f b
                      return (c, f' . g')
         Nothing → return (Nothing, Wire f . g')

instance (Monad m) ⇒ Arrow (Wire m) where
  arr fn = w where w = Wire (\a → return (Just (fn a), w))
  first w = Wire $ \(a1, a2) →
    do (b1, w') ← runWire w a1
       return (fmap (,a2) b1, first w')
  second w = Wire $ \(a1, a2) →
    do (b2, w') ← runWire w a2
       return (fmap (a1,) b2, second w')

instance (Monad m) ⇒ Alternative (Wire m a) where
  empty = w where w = Wire (const $ return (Nothing, w))
  w1 <|> w2 = Wire $ \a →
    do (b1, w1') ← runWire w1 a
       case b1 of
         Just b  → return (Just b, w1' <|> w2)
         Nothing → do (b2, w2') ← runWire w2 a
                      return (b2, w1' <|> w2')

--------------------------------------------------------------------------------

wire ∷ (Monad m) ⇒ (a → m b) → Wire m a b
wire fn = w where w = Wire (liftM (\b → (Just b, w)) . fn)

ifWire ∷ (Monad m) ⇒ Wire m Bool ()
ifWire = maybeWire (arr (\b → if b then Just () else Nothing))

maybeWire ∷ (Monad m) ⇒ Wire m a (Maybe b) → Wire m a b
maybeWire w = Wire $ \a →
  do (b, w') ← runWire w a
     return (join b, maybeWire w')

stateWire ∷ (Monad m) ⇒ Wire (StateT s m) a b → s → Wire m a b
stateWire w s = Wire $ \a →
  do ((b, w'), s') ← runStateT (runWire w a) s
     return (b, stateWire w' s')

--------------------------------------------------------------------------------

newtype Time = Time { timeSeconds ∷ Double }

delay ∷ (MonadReader Time m) ⇒ Double → Wire m a a
delay seconds = stateWire (maybeWire (wire fn)) []
  where fn a = do now   ← asks timeSeconds
                  queue ← get
                  let queue'         = queue ++ [(now + seconds, a)]
                      (nextTime, a') = head queue'
                  if now >= nextTime
                     then put (tail queue') >> return (Just a')
                     else put queue'        >> return Nothing

