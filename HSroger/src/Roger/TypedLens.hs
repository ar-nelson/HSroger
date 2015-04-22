{-# LANGUAGE FlexibleContexts      #-}
{-# LANGUAGE FlexibleInstances     #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE OverlappingInstances  #-}
{-# LANGUAGE TypeOperators         #-}
{-# LANGUAGE UnicodeSyntax         #-}

module Roger.TypedLens( Has(..)
                      , (*.)
                      , (*=)
                      , getsL
                      , setL
                      , mapL
                      , getStateL
                      , getsStateL
                      , putStateL
                      , putDebugL
                      , setStateL
                      , setDebugL
                      , mapStateL
                      , mapDebugL
                      , lens
                      , LensRecord(..)
                      , With(..)
) where

--------------------------------------------------------------------------------
-- TYPED LENSES
--
-- The "lens pattern" is a commonly-used Haskell design pattern for extensible
-- records. However, the lens library on Hackage is huge, unwieldy*, and
-- difficult to compile in my current Makefile+GHC setup.
--
-- Therefore, I've invented my own extremely-simple substitute. These typed
-- lenses are effectively HLists, which use newtypes as record field names. Lens
-- ("Has") is a typeclass, not a type, and a value of class "Has Foo" has a
-- field of type Foo.
--
-- Simple lenses are "snoc lists" (reverse cons lists), and can be constructed
-- using the `With` type operator; the first element of a `With` chain should be
-- the LensRecord type/constructor, which is equivalent to ().
--
-- * https://ro-che.info/articles/2014-04-24-lens-unidiomatic

import           Control.Monad.State

class Has τ α where
  getL ∷ α → τ
  putL ∷ τ → α → α

getsL ∷ (Has τ λ) ⇒ (τ → τ') → λ → τ'
getsL fn = fn . getL

mapL ∷ (Has τ λ) ⇒ (τ → τ) → λ → λ
mapL fn l = putL (getsL fn l) l

setL ∷ (Has τ λ) ⇒ λ → (τ' → τ) → τ' → λ
setL l constructor v = putL (constructor v) l

(*.) ∷ (Has τ λ) ⇒ λ → (τ → τ') → τ'
(*.) = flip getsL

(*=) ∷ (Has τ λ, MonadState λ μ) ⇒ (α → τ) → α → μ ()
constructor *= value = putStateL (constructor value)

getStateL ∷ (Has τ λ, MonadState λ μ) ⇒ μ τ
getStateL = gets getL

getsStateL ∷ (Has τ λ, MonadState λ μ) ⇒ (τ → α) → μ α
getsStateL f = gets (*. f)

putStateL ∷ (Has τ λ, MonadState λ μ) ⇒ τ → μ ()
putStateL = modify . putL

putDebugL ∷ (Show τ, Has τ λ, MonadState λ μ, MonadIO μ) ⇒ τ → μ ()
putDebugL v = liftIO (print v) >> putStateL v

mapStateL ∷ (Has τ λ, MonadState λ μ) ⇒ (τ → τ) → μ ()
mapStateL = modify . mapL

mapDebugL ∷ (Show τ, Has τ λ, MonadState λ μ, MonadIO μ) ⇒ (τ → τ) → μ ()
mapDebugL f = getStateL >>= putDebugL . f

setStateL ∷ (Has τ λ, MonadState λ μ) ⇒ (τ' → τ) → τ' → μ ()
setStateL c v = modify (\l → setL l c v)

setDebugL ∷ (Show τ, Has τ λ, MonadState λ μ, MonadIO μ) ⇒ (τ' → τ) → τ' → μ ()
setDebugL c v' = liftIO (print v) >> putStateL v where v = c v'

lens ∷ α → With LensRecord α
lens a = LensRecord `With` a

--------------------------------------------------------------------------------

infixl 2 `With`

data LensRecord = LensRecord

data tTail `With` tHead = tTail `With` tHead

instance Has tHead (tTail `With` tHead) where
  getL (_ `With` h) = h
  putL h (t `With` _) = t `With` h

instance Has τ tTail ⇒ Has τ (tTail `With` tHead) where
  getL (t `With` _) = getL t
  putL x (t `With` h) = putL x t `With` h

