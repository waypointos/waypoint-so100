// Card primitive for the Arm tab. Mirrors the host dashboard's Panel so the
// module's cards read identically to OVERVIEW: a `[ TITLE ]` mono-caps header
// with an optional right-aligned note and an action control, over a bordered
// surface. Kept token-for-token with the host (same --color-*, --space-*).
import type { CSSProperties, ReactNode } from 'react';
import styles from './Panel.module.css';

type Props = {
  /** Panel heading, rendered with `[ ... ]` bracket prefix in mono caps. */
  title?: string;
  /** Right-aligned note next to the heading (e.g., "live · 30 hz"). */
  note?: string;
  /** Optional control rendered at the far right of the header (after the note). */
  action?: ReactNode;
  children?: ReactNode;
  className?: string;
  style?: CSSProperties;
  /** Forwarded for test/diagnostic hooks (e.g. data-testid). */
  testId?: string;
};

export function Panel({ title, note, action, children, className, style, testId }: Props) {
  return (
    <section
      className={[styles.panel, className].filter(Boolean).join(' ')}
      style={style}
      data-testid={testId}
    >
      {title ? (
        <h3 className={styles.h}>
          <span>
            <span className={styles.hPrefix}>[</span>
            {title}
            <span className={styles.hPrefix}>]</span>
          </span>
          {note || action ? (
            <span className={styles.right}>
              {note ? <span className={styles.n}>{note}</span> : null}
              {action}
            </span>
          ) : null}
        </h3>
      ) : null}
      {children}
    </section>
  );
}
