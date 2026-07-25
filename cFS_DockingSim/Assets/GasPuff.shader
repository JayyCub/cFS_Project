Shader "Custom/GasPuff"
{
    // Soft round particle: alpha falls off radially from the sprite center, purely
    // procedural (no texture needed). Alpha-blended so overlapping puffs read as
    // translucent gas rather than glowing energy (contrast with additive ConeGlow.shader).
    Properties
    {
        _SoftPower ("Falloff Softness", Range(0.5, 4)) = 1.6
    }
    SubShader
    {
        Tags { "Queue"="Transparent" "RenderType"="Transparent" "IgnoreProjector"="True" }
        Blend SrcAlpha OneMinusSrcAlpha
        ZWrite Off
        Cull Off
        Lighting Off

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            float _SoftPower;

            struct appdata
            {
                float4 vertex : POSITION;
                float4 color  : COLOR;
                float2 uv     : TEXCOORD0;
            };

            struct v2f
            {
                float4 pos   : SV_POSITION;
                float4 color : COLOR;
                float2 uv    : TEXCOORD0;
            };

            v2f vert(appdata v)
            {
                v2f o;
                o.pos   = UnityObjectToClipPos(v.vertex);
                o.color = v.color;
                o.uv    = v.uv;
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                float dist    = length(i.uv - 0.5) * 2; // 0 at sprite center, 1 at edge
                float falloff = pow(saturate(1 - dist), _SoftPower);
                fixed4 col    = i.color;
                col.a        *= falloff;
                return col;
            }
            ENDCG
        }
    }
}
