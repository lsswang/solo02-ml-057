# Rigid perception and contact-force planning

Run `python demo.py` after installing `requirements.txt`.

The ordinary registration source fits a proper rigid transform from known point pairs by Kabsch and offers independent nearest-landmark diagnostics. It transforms contact positions and full normal/tangent frames. The ordinary contact source balances loads at a fixed selected contact collection using unconstrained bilateral normal coefficients. It does not claim unilateral friction/cap feasibility or globally choose contacts.

The fixture contains a real ordinary noncollinear registration and vertical-load balance, plus a small ambiguous point example for the extension. The task defines the probabilistic correspondence/outlier model, complete pose fitting and external friction/cap/contact-choice behavior. All data is synthetic; there is no real robot or safety claim.
