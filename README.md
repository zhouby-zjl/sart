# The Source Code of Reliable Subpath-Aware Transport Control Protocol (SART) for Wireless Multihop Networks

## What is the SART?
In energy-constrained wireless multihop networks, packet losses demand reliable and efficient transport protocols. Existing protocols rely on routing mechanisms for path adjustments but suffer from slow failure recovery. A more responsive alternative is to dynamically select high-quality subpaths for faster loss recovery while mitigating downstream congestion, yet efficiently constructing and enabling this subpath selection for transport control remains a challenge. To address this, this paper proposes SART, a subpath-aware reliable transport protocol that minimizes end-to-end packet delivery failure rate (EEFR) while maintaining low delivery time (EEDT) and high energy efficiency. SART dynamically selects high-quality per-hop subpaths based on hop distance and channel quality. By embedding subpath information in each payload, it enables relay-bypassing forwarding and retransmission via alternative subpaths for improved efficiency. Payload transmissions are dynamically adjusted to prevent downstream congestion, while implicit feedback from downstream nodes complements explicit feedback to reduce message overhead. These mechanisms enhance resilience, throughput, and energy efficiency in lossy conditions. Simulations in ndnSIM under significant noise interference demonstrate that SART outperforms existing transport schemes. In a 64-node topology with 15 nodes at 30 dB mean noise, SART achieves an EEFR of 0.02\%, an EEDT of 5.54 s, and a mean energy consumption of 0.19 J per payload receipt -- compared to at least 22.58%, 13.91 s, and 0.23 J in other schemes. Experiments across 36 to 144-node networks confirm SART’s scalability, energy efficiency, and robust performance across varying network sizes.

## How do I run the source code?
1. You need to download both the SART from github and the recent ndnSIM from https://ndnsim.net/current/. The version ndnSIM-ns-3.30.1 has been tested and is compliant with the SART code.
2. Simply copy all files of the SART to the ndnSIM folder in an override manner. 
3. You need to edit the sart-config.ini file under the ndnSIM/ns-3 folder with the correct output log path and some simulation parameters. 
4. Run SART testers. 
cd ns-3
./waf --run scratch/sart-sim --command-template="%s sart-config.ini"
5. Afterwards, you can find the simulation results under the LOG_DIR directory defined in the above ini file.
 
We are looking forward to new project opportunity in making the SART growing up. 

 *********************************************************************************
This work is licensed under CC BY-NC-SA 4.0
(https://creativecommons.org/licenses/by-nc-sa/4.0/).

Copyright (c) 2025 Boyang Zhou @ Zhejiang Lab

This file is a part of "Reliable Subpath-Aware Transport Control Protocol (SART) for Wireless Multihop Networks"
(https://github.com/zhouby-zjl/sart/).

 **********************************************************************************
