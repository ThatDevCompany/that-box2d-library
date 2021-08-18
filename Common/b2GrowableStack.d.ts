export declare class B2GrowableStack {
    m_stack: any[];
    m_count: number;
    constructor(N: number);
    Reset(): B2GrowableStack;
    Push(element: any): void;
    Pop(): any;
    GetCount(): number;
}
