/*
 * Copyright (c) 2025 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "ns3/three-gpp-channel-model.h"

#ifndef NS3_THREE_GPP_NO_FF_CHANNEL_MODEL_H
#define NS3_THREE_GPP_NO_FF_CHANNEL_MODEL_H

namespace ns3
{

class ThreeGppNoFFChannelModel : public ThreeGppChannelModel
{
  public:
    enum CalibNoFfHsn
    {
        ALL_ONES,
        CHATGPT,
        SLAGEN,
        GROK
    };

    /**
     * Constructor
     */
    ThreeGppNoFFChannelModel();

    /**
     * Destructor
     */
    ~ThreeGppNoFFChannelModel() override;

    /**
     * Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();

    Ptr<MatrixBasedChannelModel::ChannelMatrix> GetNewChannel(
        Ptr<const ThreeGppChannelParams> channelParams,
        Ptr<const ParamsTable> table3gpp,
        const Ptr<const MobilityModel> sMob,
        const Ptr<const MobilityModel> uMob,
        Ptr<const PhasedArrayModel> sAntenna,
        Ptr<const PhasedArrayModel> uAntenna) const override;

    /**
     * @brief Build a no–fast-fading channel matrix where the first cluster is
     *        filled with ones and all remaining clusters are set to zero.
     *
     * This deterministic variant constructs a channel matrix whose first cluster
     * (cluster index 0) contains a constant coefficient equal to 1 for every
     * transmit–receive antenna element pair (u,s). All other clusters are set to
     * zero. No spatial phases, antenna patterns, geometry terms, or fast-fading
     * ray structures are included.
     *
     * The resulting channel is fully correlated across all MIMO elements and
     * frequency taps, representing the simplest possible non-fading channel
     * structure for debugging, testing, or analysis. The number of clusters is
     * inherited from the provided 3GPP channel parameters to preserve compatibility
     * with downstream processing, but only the first cluster is actively populated.
     *
     * This model is primarily intended for:
     *   - testing signal-processing code paths,
     *   - validating MIMO stack logic without randomness,
     *   - establishing a strict baseline for other no–fast-fading variants.
     *
     * @note No pathloss, shadowing, or antenna pattern effects are included. These
     *       must be applied externally through the propagation loss model or other
     *       components in the simulation chain.
     *
     * @param channelParams   The 3GPP channel parameters used to determine the
     *                        number of clusters and metadata (LOS/NLOS, indices).
     * @param table3gpp       The 3GPP parameters table (not used in this simplified model).
     * @param sMob            The transmitter mobility model.
     * @param uMob            The receiver mobility model.
     * @param sAntenna        The transmitter phased-array model (only queried for size).
     * @param uAntenna        The receiver phased-array model (only queried for size).
     *
     * @return A shared pointer to a ChannelMatrix in which:
     *         - cluster 0 contains ones for all (u,s) entries,
     *         - clusters 1 ... N contain zeros.
     */
    virtual Ptr<ChannelMatrix> GetAllOnesNoFF(Ptr<const ThreeGppChannelParams> channelParams,
                                              Ptr<const ParamsTable> table3gpp,
                                              const Ptr<const MobilityModel> sMob,
                                              const Ptr<const MobilityModel> uMob,
                                              Ptr<const PhasedArrayModel> sAntenna,
                                              Ptr<const PhasedArrayModel> uAntenna) const;

    /**
     * @brief Build a deterministic no–fast-fading channel matrix preserving both
     *        spatial phases and antenna pattern amplitudes (ChatGPT model).
     *
     * This variant generates a fully deterministic, 3GPP-consistent array response
     * for the direct path by combining:
     *   - per-element antenna positions (geometry-based phase),
     *   - per-element antenna field patterns (theta/theta and phi/phi terms),
     *   - propagation phase due to TX–RX separation.
     *
     * Unlike simplified models, both phase and amplitude contributions from the
     * antenna patterns are retained, enabling realistic directional gain behavior
     * and beamforming characteristics while eliminating fast fading and random ray
     * construction.
     *
     *
     * This model is best suited for realistic indoor/outdoor downlink geometry
     * studies, deterministic MIMO evaluations, and scenarios requiring physically
     * meaningful directional array behavior without small-scale fading.
     *
     * @param channelParams   The 3GPP channel parameters (large-scale effects,
     *                        angles of departure/arrival, LOS/NLOS condition).
     * @param table3gpp       The 3GPP parameters table (unused for deterministic variant).
     * @param sMob            The transmitter mobility model.
     * @param uMob            The receiver mobility model.
     * @param sAntenna        The transmitter phased-array model.
     * @param uAntenna        The receiver phased-array model.
     *
     * @return A shared pointer to a ChannelMatrix containing a single deterministic
     *         cluster with full spatial structure (phases + pattern amplitudes),
     *         normalized to a well-defined channel norm.
     */
    Ptr<MatrixBasedChannelModel::ChannelMatrix> GetChatGptNoFF(
        Ptr<const ThreeGppChannelParams> channelParams,
        Ptr<const ParamsTable> table3gpp,
        const Ptr<const MobilityModel> sMob,
        const Ptr<const MobilityModel> uMob,
        Ptr<const PhasedArrayModel> sAntenna,
        Ptr<const PhasedArrayModel> uAntenna) const;
    /**
     * @brief Build a deterministic no–fast-fading channel matrix where all MIMO
     *        elements share the same complex coefficient (SLagen model).
     *
     * This variant constructs a single deterministic complex coefficient using the
     * centroid positions of the transmit and receive arrays together with the
     * direct-path angles of departure and arrival. The coefficient is replicated
     * uniformly across all (u,s) antenna element pairs, yielding a fully correlated
     * MIMO channel. No fast fading, ray-level randomness, or per-element phase
     * variations are included.
     *
     * Note: this model is only valid when all the antenna elements have the same
     * polarization. For example, such an assumption is typically used to compute
     * the DL geometry in calibration scenarios, based on a single antenna port.
     *
     *
     * This model is most suitable for simplified geometry checks or for scenarios
     * where a scalar effective channel is desired across a MIMO array.
     *
     * @param channelParams   The 3GPP channel parameters (large-scale effects,
     *                        angles of departure/arrival, LOS/NLOS condition).
     * @param table3gpp       The 3GPP parameters table (unused for deterministic variant).
     * @param sMob            The transmitter mobility model.
     * @param uMob            The receiver mobility model.
     * @param sAntenna        The transmitter phased-array model.
     * @param uAntenna        The receiver phased-array model.
     *
     * @return A shared pointer to a ChannelMatrix containing a single deterministic
     *         cluster with uniform coefficients across all antenna elements.
     */
    Ptr<MatrixBasedChannelModel::ChannelMatrix> GetSLagenNoFF(
        Ptr<const ThreeGppChannelParams> channelParams,
        Ptr<const ParamsTable> table3gpp,
        const Ptr<const MobilityModel> sMob,
        const Ptr<const MobilityModel> uMob,
        Ptr<const PhasedArrayModel> sAntenna,
        Ptr<const PhasedArrayModel> uAntenna) const;
    /**
     * @brief Build a deterministic no–fast-fading channel matrix preserving
     *        spatial phase structure but normalizing pattern amplitudes (Grok model).
     *
     * This variant constructs the channel from the direct-path angles of departure
     * and arrival together with the exact physical positions of the transmit and
     * receive antenna elements. Per-element phase shifts (geometry-based) are kept,
     * enabling directional beamforming effects without fast fading.
     *
     * The element-level field patterns are used only for their phase contribution:
     * the magnitude of the co-pol (theta-theta minus phi-phi) term is normalized
     * out to remove amplitude variation. Thus, all directions have equal intrinsic
     * per-element gain while maintaining realistic phase aperture behavior.
     *
     *
     * This model is suitable for controlled geometry tests, beamforming logic
     * evaluation, or scenarios where spatial phases are desired without the
     * directional gain shaping of antenna patterns.
     *
     * @param channelParams   The 3GPP channel parameters (large-scale effects,
     *                        angles of departure/arrival, LOS/NLOS condition).
     * @param table3gpp       The 3GPP parameters table (unused for deterministic variant).
     * @param sMob            The transmitter mobility model.
     * @param uMob            The receiver mobility model.
     * @param sAntenna        The transmitter phased-array model.
     * @param uAntenna        The receiver phased-array model.
     *
     * @return A shared pointer to a ChannelMatrix containing a single deterministic
     *         cluster with per-element spatial phases and normalized pattern
     *         amplitudes.
     */
    Ptr<MatrixBasedChannelModel::ChannelMatrix> GetGrokNoFF(
        Ptr<const ThreeGppChannelParams> channelParams,
        Ptr<const ParamsTable> table3gpp,
        const Ptr<const MobilityModel> sMob,
        const Ptr<const MobilityModel> uMob,
        Ptr<const PhasedArrayModel> sAntenna,
        Ptr<const PhasedArrayModel> uAntenna) const;

    CalibNoFfHsn m_noFfModel{ALL_ONES};
};
} // namespace ns3

#endif // NS3_THREE_GPP_NO_FF_CHANNEL_MODEL_H
