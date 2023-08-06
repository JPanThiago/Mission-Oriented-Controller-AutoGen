function ref_resampled = resample_ref(ref)
    % resamples data with a desired time step for soft robot (cited from Bruder et al. "ksysid.m")
    tr = 0 : 0.083023892317701 : ref.t(end);
    ref_resampled = interp1(ref.t , ref.y , tr);
end