# References and source status

Every value or relation in the repo is tagged with one of these source states:
- **[read]**: I opened the source and the cited value comes from it.
- **[abstract]**: only an abstract or search snippet was available.
- **[std]**: a standard textbook result quoted from memory. The section is named, but the page was not re-checked.
- **[assumed]**: a modelling choice of this repo. These are listed in the dossier, section 8.2.

## Systems engineering

- **[W]** C. S. Wasson, *System Engineering Analysis, Design, and Development: Concepts,
  Principles, and Practices*, 2nd ed., Wiley, 2016. **[read]**, ch. 4–7, 13, 19–22, 26–28, 31–34.
  Reading notes with printed and PDF page numbers are in `wasson_extract_ch04-22.md` and
  `wasson_extract_ch26-34.md`.

## Extrusion physics and rubber processing

- **[S1]** C. Rauwendaal, *Polymer Extrusion*, 5th ed., Hanser, 2014. **[std]**
  - Ch. 7: melt conveying, drag and pressure flow, power consumption in the metering section.
  - Ch. 7: the apparent-viscosity approximation at the channel shear rate.
- **[S2]** Z. Tadmor, C. G. Gogos, *Principles of Polymer Processing*, 2nd ed., Wiley, 2006.
  **[std]**
  - Sec. 6.3: the single-screw pump with the flat-plate model, the shape factors Fd and Fp, and power.
  - Ch. 9: starve-fed and partially filled screws.
- **[S3]** J. L. White, *Rubber Processing: Technology, Materials, and Principles*, Hanser, 1995.
  **[std]** Covers the background on cold-feed rubber extruders and on wall slip in rubber compounds.
  No numbers are taken from it.
- **[S4]** Brzoskowski et al., "Experimental and theoretical study of the flow characteristics of
  rubber compounds in an extruder screw", *International Polymer Processing* 2 (1987),
  doi:10.1515/ipp-1987-0004. **[abstract]**
  - Used for: cold-feed tread compounds were modelled as pseudoplastic with a yield value, and a
    one-dimensional model predicted flow and pressure only once starvation (partial fill) was included.
  - This is the basis for the partially filled screw model here.
- **[S5]** Vendor listing for a 150 mm cold-feed rubber extruder (XJW-150 class), found by web
  search on 9 Nov 2026. **[abstract]**
  - Values used: screw speed 4.5–45 rpm, 185 kW drive, design maximum output 1100 kg/h at the
    highest speed with no head pressure.
  - This is a vendor catalogue, so it sets the size class of the reference machine and nothing more.
- **[S6]** "Optimization Approaches in Rubber Extrusion for Energy and CO2 Savings", *KGK Kautschuk
  Gummi Kunststoffe* (rubberpoint.de), found 9 Nov 2026. **[abstract]**
  - Used for: specific energy 0.3–1.7 kWh/kg on a 40 mm, 16 L/D rubber extruder.
- **[S7]** Y.-S. Ha, J.-R. Cho, T.-H. Kim, J.-H. Kim, "Finite element analysis of rubber extrusion
  forming process" (in Korean, English abstract), KSME conference, 2007. **[read]**, Table 1.
  - Compound w6043: power law k = 0.9881e6, n = 0.2226; temperature coefficient 0.8917e-2;
    reference temperature 363 K.
  - The extracted text does not state the units. SI units are assumed.
- **[S8]** F. Previdi, S. M. Savaresi, A. Panarotto, "Design of a feedback control system for real-time
  control of flow in a single-screw extruder", IFAC World Congress, 2005. **[read]**
  - Pressure is controlled by screw speed through the inverter voltage.
  - Identified model: Gp(s) = 4.9 e^{-0.1472 s}/(1 + 1.2028 s) bar/V, on an OMP T35 extruder with polypropylene.
  - The pressure oscillation is about 2 bar on 22 bar before control.
  - The PID was tuned by the area method.
  - The journal version is Control Engineering Practice 14 (2006) 1111–1121 **[abstract]**.
- **[S9]** ASTM D5099, *Standard Test Methods for Rubber: Measurement of Processing Properties Using
  Capillary Rheometry*. **[abstract]** (title and scope only). It is the route to measure the plant
  compound's power-law constants.

## Control and estimation

- **[C1]** K. J. Åström, B. Wittenmark, *Computer-Controlled Systems: Theory and Design*, 3rd ed.,
  Prentice Hall, 1997. **[std]**
  - Ch. 5: pole placement, the polynomial approach and the Diophantine equation.
  - Ch. 8: digital PID.
  - Sec. 9.4: anti-windup for polynomial controllers through an observer polynomial.
- **[C2]** I. D. Landau, G. Zito, *Digital Control Systems: Design, Identification and
  Implementation*, Springer, 2006. **[std]**
  - Ch. 2–3: the RST structure, the fixed parts Hs and Hr, and robustness margins (modulus margin of
    at least 0.5, delay margin of at least Ts).
- **[C3]** V. Bobál, J. Böhm, J. Fessl, J. Macháček, *Digital Self-tuning Controllers*, Springer,
  2005. **[std]** Covers polynomial controllers for PLC-class targets. Bobál is at Tomas Bata University in Zlín (UTB).
- **[C4]** K. J. Åström, T. Hägglund, *Advanced PID Control*, ISA, 2006. **[std]** Covers Ms as the
  robustness measure, back-calculation anti-windup, and setpoint weighting.
- **[C5]** E. F. Camacho, C. Bordons, *Model Predictive Control*, 2nd ed., Springer, 2007, Ch. 2 (DMC). **[std]**
- **[C6]** P. Lundström, J. H. Lee, M. Morari, S. Skogestad, "Limitations of dynamic matrix control",
  *Computers & Chemical Engineering* 19 (1995) 409–421. **[std]** Shows that DMC's output-step
  disturbance model is slow for input disturbances on lag-dominant plants.
- **[C7]** L. Ljung, *System Identification: Theory for the User*, 2nd ed., Prentice Hall, 1999. **[std]**
  - Sec. 4.2: the output-error (OE) structure.
  - Ch. 7–8: prediction-error methods and their consistency.
- **[C8]** D. Simon, *Optimal State Estimation*, Wiley, 2006, Ch. 13 (extended Kalman filter). **[std]**
- **[C9]** S. Skogestad, "Simple analytic rules for model reduction and PID controller tuning",
  *J. Process Control* 13 (2003) 291–309. **[std]**

## PLC

- **[P1]** Siemens data sheet 6AG1513-1AL02-2AB0, SIPLUS S7-1500 CPU 1513-1 PN, "based on
  6ES7513-1AL02-0AB0", dated 10 Apr 2023. **[read]**
  - CPU processing times, typical: 40 ns per bit operation, 48 ns per word operation, 64 ns per
    fixed-point operation, 256 ns per floating-point operation.
  - Work memory: 300 kbyte for program, 1.5 Mbyte for data.
- **[P2]** Siemens TIA Portal V18 with STEP 7 Professional. The compile results come from this
  installation through TIA Openness.
- **[C10]** R. J. LeVeque, *Finite Volume Methods for Hyperbolic Problems*, Cambridge University
  Press, 2002, Ch. 6 (high-resolution methods, flux limiters, superbee). **[std]**
