import { createRoot } from "react-dom/client";
import "../app/globals.css";
import { SimulatorApp } from "../components/SimulatorApp";

const root = document.getElementById("root");
if (!root) throw new Error("Application root element was not found.");

createRoot(root).render(<SimulatorApp />);
