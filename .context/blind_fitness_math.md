# Blind Fitness Math Reference

## 1. Normalized Innovation Squared (NIS)
Used to measure internal filter consistency without ground truth.
Formula:
$NIS(k) = \nu(k)^T S(k)^{-1} \nu(k)$
Where:
- $\nu(k)$ is the measurement residual ($y - \hat{y}$).
- $S(k)$ is the innovation covariance.

## 2. Peer Agreement Cost ($J_{PA}$)
Used to measure consensus, weighted by trust.
Formula:
$J_{PA} = \frac{\sum (w_{ij} \cdot d_{ij})}{\sum w_{ij}}$
Where:
- $d_{ij}$ is the state distance between Agent $i$ and Neighbor $j$.
- $w_{ij}$ is the reputation weight from `AdaptiveState` (0.0 to 1.0).

## 3. Bandwidth Cost ($J_{BW}$)
Used to penalize excessive network usage.
Formula:
$J_{BW} = \frac{\text{BytesSent}}{\text{Budget}}$

## 4. Composite Fitness Function
The objective function to **minimize**.
$J_{fitness} = w_{nis} \cdot \text{Avg}(NIS) + w_{peer} \cdot J_{PA} + w_{bw} \cdot J_{BW}$