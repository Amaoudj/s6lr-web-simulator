import type { ReactNode } from "react";
import "./globals.css";

export const metadata = {
  title: "S6LR // Kinematics Lab",
  description:
    "Interactive 18-DOF S6LR hexapod kinematics and gait simulator using the original robot meshes.",
};

export default function RootLayout({ children }: { children: ReactNode }) {
  return (
    <html lang="en">
      <head>
        <link rel="icon" href="/favicon.svg" type="image/svg+xml" />
        <meta name="theme-color" content="#171a19" />
      </head>
      <body>{children}</body>
    </html>
  );
}

