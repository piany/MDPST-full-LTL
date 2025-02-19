# MDPST-full-LTL
Planning with Linear Temporal Logic Specifications: Handling Quantifiable and Unquantifiable Uncertainty

-----
Description
-----
This package contains the implementation for optimal robust policy synthesis algorithms given a  Markov Decision Process with Set-Valued Transitions (MDPST) (as the robotic systems model under both quantificable and unquantificable uncertainties) and a (full) Linear Temporal Logic (LTL) formula (as the robot task specification). It outputs a stationary  and finite-memory policy consists of plan prefix and plan suffix, such that the controlled robot behavior fulfills the LTL task with a maximal probability.

Features
-----
* Utilises MDPSTs as the modelling framework for robot planning under both quantifiable and unquantifiable uncertainty
* Both Limit-Deterministic Buchi Automaton (LDBA) and Deterministic Rabin Automaton (DRA) are implemented as the automaton translation of LTL formula
* Computing Winning Region (WR) of MDPSTs
* Robust value iteration is implemented for optimal policy synthesis of MDPSTs

* ----
Dependence
----
* Install python packages like Networkx, numpy, ortools
* Compile [ltl2ba executable](http://www.lsv.ens-cachan.fr/%7Egastin/ltl2ba/download.php) for your OS.
* Compile [ltl2dstar executable](http://www.ltl2dstar.de) for your OS.
* Compile [ltl2ldba executable](https://www7.in.tum.de/~kretinsk/rabinizer4.html) for your OS.  

* In ltl2dra.py, def run_ltl2dra(formula), remember to replace the directories "ltl2dra_dir" and "ltl2ba_dir" with you own 
* In automaton.py, def ltl2auto(self,ltl), remember to replace the directory "out" with your own
