import type { ReactNode, SVGProps } from 'react';

function Svg({ children, ...rest }: SVGProps<SVGSVGElement> & { children: ReactNode }) {
  return (
    <svg
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth={2.4}
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
      {...rest}
    >
      {children}
    </svg>
  );
}

export const MicIcon = (p: SVGProps<SVGSVGElement>) => (
  <Svg {...p}>
    <rect x="9" y="3" width="6" height="11" rx="3" fill="currentColor" stroke="none" />
    <path d="M5.5 11.5a6.5 6.5 0 0 0 13 0M12 18v3" />
  </Svg>
);

export const KeyboardIcon = (p: SVGProps<SVGSVGElement>) => (
  <Svg {...p}>
    <rect x="2.5" y="6" width="19" height="12" rx="2.5" />
    <path d="M7 10h.01M11 10h.01M15 10h.01M8 14h8" />
  </Svg>
);

export const SendIcon = (p: SVGProps<SVGSVGElement>) => (
  <Svg {...p}>
    <path d="M12 19V5M5.5 11.5 12 5l6.5 6.5" />
  </Svg>
);

export const CheckIcon = (p: SVGProps<SVGSVGElement>) => (
  <Svg {...p}>
    <path d="M4.5 12.5 10 18l9.5-11" />
  </Svg>
);

export const CrossIcon = (p: SVGProps<SVGSVGElement>) => (
  <Svg {...p}>
    <path d="M6 6l12 12M18 6 6 18" />
  </Svg>
);

export const WarnIcon = (p: SVGProps<SVGSVGElement>) => (
  <Svg {...p}>
    <path d="M12 3.5 2.5 20h19L12 3.5Z" />
    <path d="M12 10v4.5M12 17.2v.1" />
  </Svg>
);

export const DownIcon = (p: SVGProps<SVGSVGElement>) => (
  <Svg {...p}>
    <path d="M12 5v14M5.5 12.5 12 19l6.5-6.5" />
  </Svg>
);

export const ViewIcon = (p: SVGProps<SVGSVGElement>) => (
  <Svg {...p}>
    <circle cx="12" cy="12" r="3" />
    <path d="M12 2.5v4M12 17.5v4M2.5 12h4M17.5 12h4" />
  </Svg>
);

export const GhostIcon = (p: SVGProps<SVGSVGElement>) => (
  <Svg {...p}>
    <path d="M5 20.5V11a7 7 0 0 1 14 0v9.5l-2.4-2-2.3 2-2.3-2-2.3 2-2.3-2L5 20.5Z" strokeDasharray="2.6 2.6" />
  </Svg>
);

export const CameraOffIcon = (p: SVGProps<SVGSVGElement>) => (
  <Svg {...p}>
    <path d="M3 3l18 18M9.5 6H15l1.5 2H19a2 2 0 0 1 2 2v7.5M17 19H5a2 2 0 0 1-2-2v-7a2 2 0 0 1 2-2h1" />
  </Svg>
);

export const SoundOffIcon = (p: SVGProps<SVGSVGElement>) => (
  <Svg {...p}>
    <path d="M4 9.5h3.5L12 5.5v13l-4.5-4H4v-5ZM16 9.5l5 5M21 9.5l-5 5" />
  </Svg>
);
