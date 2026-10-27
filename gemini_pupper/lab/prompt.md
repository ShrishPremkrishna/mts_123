<!--
Gemini's system prompt: who Pupper is and how it behaves. The robot server sends
this file to the web app when it connects (comments like this one are removed).
After editing, reconnect in the app: no rebuild or restart needed.

You write it in two passes. Replace every TODO line with your own words; keep
the headings. Short sentences work best out loud.

For each rule you write, ask: is a sentence here enough, or does the robot also
have to make sure of it in code? (Part 3 and the reflection ask you about this.)

What the tests check (they can only look for words; you check the behavior
by talking to Pupper). A section still counts as unwritten while it says TODO.
- TODO 1: the first three sections have 60+ words together, and say "tool".
    Passes: "You can only move by calling a tool."
- TODO 6: Follow mode says "end" or "stop".
    Passes: "Any motion ends follow mode. Before you call that tool, say so."
- TODO 11: Tricks and Safety say "stop", "fall" (or "fell"), and "order"
  (or "row", "sequence").
    Passes: "If someone says stop, call stop right away."
            "If you fall over, ask a person to set you upright."
            "If the person asks for several things in a row, call the tools in that order."
-->

# Who you are

TODO 1: Who is Pupper? A personality, and how it talks. How does it stay
Pupper when someone asks for something it can't do? (A first draft is fine:
in Part 7 you come back here and make Pupper a real character.)

# How you move

TODO 1: Pupper can only move by calling a tool. What should it do first: call
the tool, or talk? What must it never say?

# When a tool says no

TODO 1: Every tool reply says what really happened. What should Pupper do when
a tool refuses or fails? What should it never pretend?

# Follow mode

TODO 6: What keeps follow mode on, what ends it, and what Pupper says before a
motion ends it.

# Tricks

TODO 11: Tricks start standing or lying down, and keep their final pose. How
should Pupper use play_trick and interactive_action? What if someone asks for
several things in a row? And for a trick Pupper doesn't have?

# Safety

TODO 11: What Pupper does if someone says stop, and if it falls over (it cannot
get up by itself).
